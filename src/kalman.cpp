
#include "arm_math.h"
#include "math_helper.h"
#if defined(SEMIHOSTING)
#include <stdio.h>
#endif
#define SNR_THRESHOLD 90

namespace kalman
{
  /* X: etat

    *  P: Cov état
    *  F: systeme linearisé
    *  B,U: mat commande et commande
    *  Q: bruit sur commande
    *  *_data ou calc* indique un buffer de float32_t, *_c indique une arm_matrix:pour calculs
    */

  //public:
  const uint16_t dim_etat = 3, dim_cmde = 2; //modifier ici si état ou commande de taille différente
  const uint16_t dim_mesure1 = 3;            //ajouter ici si mesure de taille différente + ajouter H_data(estimation mesure) si linéaire + ajouter buffers calculs+ modifier/ajouter kalman_maj(mise à jour)
  float32_t *X = NULL;                       //état du systeme, alias X_data: x,y,theta
  float32_t *P = NULL;                       //idem P_A, P_B
  float32_t *U = NULL;                       //commande
  float32_t Dt = 0.01;                       //intervalle de temps

  //privé:
  arm_status statut;

  //à créer, ou déjà en mémoire? -> supprimer les allocations statiques de X,P,U???
  float32_t U_data[dim_cmde];
  arm_matrix_instance_f32 *U_c;
  float32_t Mesure1_data[dim_mesure1];
  arm_matrix_instance_f32 *Mesure1_c;
  /* 
  
  
  
  état du système */
  float32_t X_data[dim_etat * 1] =
      {
          /* état initial du système */
          0.f, 0.f, 0.f};
  arm_matrix_instance_f32 *X_c;

  float32_t P_data[dim_etat * dim_etat] =
      {
          /* matrice de covariance initiale de l'état */
          1.f, 0.f, 0.f,
          0.f, 1.f, 0.f,
          0.f, 0.f, 1.f};
  arm_matrix_instance_f32 *P_c;

  /* 
  
  
  
  description du systeme */

  const float32_t B_data[dim_etat * dim_cmde] =
      {
          /* matrice de commande */
          1.f, 0.f,
          0.f, 1.f};
  arm_matrix_instance_f32 *B;
  arm_matrix_instance_f32 *BT;
  // B non utilisée car non linéaire. --> à supprimer

  float32_t F_data[dim_etat * dim_etat] =
      {
          /* systeme linearisé */
          1.f, 0.f, 0.f,
          0.f, 1.f, 0.f,
          0.f, 0.f, 1.f};
  arm_matrix_instance_f32 *F;
  arm_matrix_instance_f32 *FT;

  float32_t H1_data[dim_mesure1 * dim_etat] =
      {
          /* matrice d'estimation de la mesure à partir du vecteur d'état*/
          1.f, 0.f, 0.f,
          0.f, 1.f, 0.f,
          0.f, 0.f, 1.f};
  arm_matrix_instance_f32 *H1;
  arm_matrix_instance_f32 *H1T;

  float32_t R1_data[dim_mesure1 * dim_mesure1] =
      {
          /* matrice de bruit de la mesure */
          1.f, 0.f, 0.f,
          0.f, 1.f, 0.f,
          0.f, 0.f, 1.f};
  arm_matrix_instance_f32 *R1;

  const float32_t Q_data[dim_cmde * dim_cmde] =
      {
          1.0};
  arm_matrix_instance_f32 *Q;

  /* ----------------------------------------------------------------------


    * Temporary buffers  for storing intermediate values
    * ------------------------------------------------------------------- */

  /* buffers de calculs */
  // taille X
  float32_t calc1_data[dim_etat];
  float32_t calc2_data[dim_etat];
  arm_matrix_instance_f32 *calc1;
  arm_matrix_instance_f32 *calc2;
  // taille P
  float32_t calc3_data[dim_etat * dim_etat];
  float32_t calc4_data[dim_etat * dim_etat];
  arm_matrix_instance_f32 *calc3;
  arm_matrix_instance_f32 *calc4;
  // taille cmde*dim_etat
  float32_t calc5_data[dim_cmde * dim_etat];
  arm_matrix_instance_f32 *calc5;
  // taille dim_mesure1
  float32_t calc6_data[dim_mesure1];
  float32_t calc7_data[dim_mesure1];
  arm_matrix_instance_f32 *calc6;
  arm_matrix_instance_f32 *calc7;
  // taille dim_etat*dim_mesure1
  float32_t calc8_data[dim_etat * dim_mesure1];
  arm_matrix_instance_f32 *calc8;
  // taille dim_mesure1*dim_mesure1
  float32_t calc9_data[dim_mesure1 * dim_mesure1];
  float32_t calc10_data[dim_mesure1 * dim_mesure1];
  arm_matrix_instance_f32 *calc9;
  arm_matrix_instance_f32 *calc10;

  void init()
  {
    arm_mat_init_f32(X_c, dim_etat, 1, X_data);
    arm_mat_init_f32(U_c, dim_cmde, 1, U_data);
    arm_mat_init_f32(P_c, dim_etat, dim_etat, P_data);
    X = X_data;
    P = P_data;

    arm_mat_init_f32(F, dim_etat, dim_etat, (float32_t *)F_data);
    arm_mat_trans_f32(F, FT);
    arm_mat_init_f32(Q, dim_cmde, dim_cmde, (float32_t *)Q_data);
    arm_mat_init_f32(H1, dim_etat, dim_mesure1, H1_data);
    arm_mat_trans_f32(H1, H1T);
    arm_mat_init_f32(R1, dim_mesure1, dim_mesure1, R1_data);

    arm_mat_init_f32(calc1, dim_etat, 1, calc1_data);
    arm_mat_init_f32(calc2, dim_etat, 1, calc2_data);
    arm_mat_init_f32(calc3, dim_etat, dim_etat, calc3_data);
    arm_mat_init_f32(calc4, dim_etat, dim_etat, calc4_data);
    arm_mat_init_f32(calc5, dim_cmde, dim_etat, calc5_data);
    arm_mat_init_f32(calc6, dim_mesure1, 1, calc6_data);
    arm_mat_init_f32(calc7, dim_mesure1, 1, calc7_data);
    arm_mat_init_f32(calc8, dim_etat, dim_mesure1, calc8_data);
    arm_mat_init_f32(calc9, dim_mesure1, dim_mesure1, calc9_data);
    arm_mat_init_f32(calc10, dim_mesure1, dim_mesure1, calc10_data);
  }

  void kalman_predict()
  {
    /* 
    * Prédiction de la position (= état du système) à l'instant t+Dt à partir de la consigne en vitesse U.
    * Rappel (ou pas): U=[U[0], U[1]]=[vitesse, vitesse de rotation]
    *                  X=[x, y, theta]=[abscisse, ordonnée, angle/orientation du robot]
    *                  B: non utilisé, système non linéaire en commande
    */

    //Xkp1m = F*Xk +B*U
    //statut=arm_mat_mult_f32(F, X_c, calc1);
    //statut=arm_mat_mult_f32(B, U_c, calc2);
    //statut=arm_mat_add_f32(calc1, calc2, X_c);
    X[0] += U[0] * Dt * cos(X[2]);
    X[1] += U[0] * Dt * sin(X[2]);
    X[2] += U[1] * Dt;

    F_data[2] = -U[0] * Dt * sin(X[2]);
    F_data[dim_etat + 2] = -U[0] * Dt * cos(X[2]);

    //Pkp1m = F*(Pk*FT) + Q
    statut = arm_mat_mult_f32(P_c, FT, calc3);
    statut = arm_mat_mult_f32(F, calc3, calc4);
    statut = arm_mat_add_f32(Q, calc4, P_c);
    //débug: vérifier si statut = ARM_MATH_SUCCESS;
  }

  void kalman_maj(arm_mat_instance_f32 *Mesure_c, arm_mat_instance_f32 *H = H1, arm_mat_instance_f32 *R = R1)
  {
    /* 
    * Met à jour X et P à partir de Mesure, pointeur vers mesure de position.
    * Mesure est un pointeur, issu indifféremment de l'odométrie ou de la caméra 
    * H : matrice d'observation, par défaut à H1 (=identité)
    */
    arm_mat_mult_f32(H, X_c, calc6);
    arm_mat_sub_f32(Mesure_c, calc6, calc7); //calcul résidu

    arm_mat_mult_f32(P_c, H1T, calc8);  //calc8=P*Ht
    arm_mat_mult_f32(H1, calc8, calc9); //calc9=H*P*Ht

    arm_mat_add_f32(calc9, R1, calc10); //S = H*P*Ht+R1
    arm_mat_inv_f32(calc10, calc9);
    //à poursuivre... pas le temps aujourd'hui ..
    arm_mat_mult_f32(calc8, calc9, ); //calcul gain Kalman
  }
} // namespace kalman
