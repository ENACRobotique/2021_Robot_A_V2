

/*#if defined(SEMIHOSTING)
#include <stdio.h>
#endif*/


//vérifier que les tailles des matrices sont correctes lors des opérations -> à vérifier avec statut=<operation>



extern int variable_c;


#ifndef KALMAN_H_
#define KALMAN_H_

#define ARM_MATH_MATRIX_CHECK 
#define SNR_THRESHOLD 90
#include "arm_math.h"
#include "math_helper.h"

class kalman
{
    /* X: etat

    *  P: Cov état
    *  F: systeme linearisé
    *  B,U: mat commande et commande
    *  Q: bruit sur commande
    *  *_data ou calc* indique un buffer de float32_t, *_c indique une arm_matrix:pour calculs
    */

    
//    const uint16_t dim_etat = 3, dim_cmde = 2; //modifier ici si état ou commande de taille différente
//    const uint16_t dim_mesure1 = 3;            //ajouter ici si mesure de taille différente + ajouter H_data(estimation mesure) si linéaire + ajouter buffers calculs+ modifier/ajouter kalman_maj(mise à jour)
#define dim_etat 3
#define dim_cmde 2
#define dim_mesure1 3
public:
    float32_t *X = NULL;                       //état du systeme, alias X_data: x,y,theta
    float32_t *P = NULL;                       //idem P_A, P_B
    float32_t *U = NULL;                       //commande : V, omega
    float32_t Dt = 0.1;                       //intervalle de temps, à voir selon metro


    float M1largeurbb=0.1;                //cas d'un bruit blanc uniforme : erreur mesure1 (cf R1_data)
    float M1var=M1largeurbb/6;             //theta -> erreur 7,5°; x, y -> erreur 0.13
    // erreur commande : cf Q_data
    
    //privé:
    arm_status statut;

    //à créer, ou déjà en mémoire? -> supprimer les allocations statiques de X,P,U???
    float32_t U_data[dim_cmde];
    arm_matrix_instance_f32 *U_c = new arm_matrix_instance_f32;
    float32_t Mesure1_data[dim_mesure1];
    arm_matrix_instance_f32 *Mesure1_c = new arm_matrix_instance_f32;
    /* 



    état du système */
private:
    float32_t X_data[dim_etat * 1] =
        {
            /* état initial du système */
            0.f, 0.f, 0.f};
    arm_matrix_instance_f32 *X_c = new arm_matrix_instance_f32;

    float32_t P_data[dim_etat * dim_etat] =
        {
            /* matrice de covariance initiale de l'état */
            2.f, 0.f, 0.f,
            0.f, 2.f, 0.f,
            0.f, 0.f, 0.2f};
    arm_matrix_instance_f32 *P_c = new arm_matrix_instance_f32;

    /* 



    description du systeme */

    float32_t B_data[dim_etat * dim_cmde] =
        {
            /* matrice de commande */
            1.f, 0.f,
            0.f, 0.f,
            0.f, 1.0f};
    arm_matrix_instance_f32 *B = new arm_matrix_instance_f32;

    float32_t G_data[dim_etat * dim_cmde] =
        {
            /* matrice de commande */
            1.f, 0.f,
            1.f, 0.f,
            0.f, 1.0f};
    arm_matrix_instance_f32 *G = new arm_matrix_instance_f32;
    float32_t GT_data[dim_etat * dim_cmde];
    arm_matrix_instance_f32 *GT= new arm_matrix_instance_f32;
    // B non linéaire. --> modifié dans kalman_pred

    float32_t F_data[dim_etat * dim_etat] =
        {
            /* systeme linearisé */
            1.f, 0.f, 0.f,
            0.f, 1.f, 0.f,
            0.f, 0.f, 1.f};
    arm_matrix_instance_f32 *F = new arm_matrix_instance_f32;
    float32_t FT_data[dim_etat * dim_etat];
    arm_matrix_instance_f32 *FT = new arm_matrix_instance_f32;

    float32_t H1_data[dim_mesure1 * dim_etat] =
        {
            /* matrice d'estimation de la mesure à partir du vecteur d'état*/
            1.f, 0.f, 0.f,
            0.f, 1.f, 0.f,
            0.f, 0.f, 1.f};
    arm_matrix_instance_f32 *H1 = new arm_matrix_instance_f32;
    float32_t H1T_data[dim_mesure1 * dim_etat];
    arm_matrix_instance_f32 *H1T = new arm_matrix_instance_f32;
    
    float32_t R1_data[dim_mesure1 * dim_mesure1] =
        {
            /* matrice de bruit de la mesure */
            M1var, 0.f, 0.f,
            0.f, M1var, 0.f,
            0.f, 0.f, M1var};
    arm_matrix_instance_f32 *R1 = new arm_matrix_instance_f32;

    const float32_t Q_data[dim_cmde * dim_cmde] =
        {
            1.f, 0.f,
            0.f, 0.05f};
    arm_matrix_instance_f32 *Q = new arm_matrix_instance_f32;


    /* 



    utilitaires */
    float32_t Id_etat_data[dim_etat * dim_etat] =
        {
            /* matrice identité de la dimension de l'état*/
            1.f, 0.f, 0.f,
            0.f, 1.f, 0.f,
            0.f, 0.f, 1.f};
    arm_matrix_instance_f32 *Id_etat = new arm_matrix_instance_f32;



    /* ------------------------------------------------------------------
    * Temporary buffers  for storing intermediate values
    * ------------------------------------------------------------------- */

    /* buffers de calculs */
    // taille X
    float32_t calc1_data[dim_etat];
    float32_t calc2_data[dim_etat];
    arm_matrix_instance_f32 *calc1 = new arm_matrix_instance_f32;
    arm_matrix_instance_f32 *calc2 = new arm_matrix_instance_f32;
    // taille P
    float32_t calc3_data[dim_etat * dim_etat];
    float32_t calc4_data[dim_etat * dim_etat];
    arm_matrix_instance_f32 *calc3 = new arm_matrix_instance_f32;
    arm_matrix_instance_f32 *calc4 = new arm_matrix_instance_f32;
    // taille cmde*dim_etat
    float32_t calc5_data[dim_cmde * dim_etat];
    arm_matrix_instance_f32 *calc5 = new arm_matrix_instance_f32;
    // taille dim_mesure1
    float32_t calc6_data[dim_mesure1];
    float32_t calc7_data[dim_mesure1];
    arm_matrix_instance_f32 *calc6= new arm_matrix_instance_f32;
    arm_matrix_instance_f32 *calc7= new arm_matrix_instance_f32;
    // taille dim_etat*dim_mesure1
    float32_t calc8_data[dim_etat * dim_mesure1];
    float32_t calc8b_data[dim_etat * dim_mesure1];
    arm_matrix_instance_f32 *calc8= new arm_matrix_instance_f32;
    arm_matrix_instance_f32 *calc8b= new arm_matrix_instance_f32;
    // taille dim_mesure1*dim_mesure1
    float32_t calc9_data[dim_mesure1 * dim_mesure1];
    float32_t calc10_data[dim_mesure1 * dim_mesure1];
    arm_matrix_instance_f32 *calc9= new arm_matrix_instance_f32;
    arm_matrix_instance_f32 *calc10= new arm_matrix_instance_f32;
    //taille dim_etat*dim_cmde
    float32_t calc11_data[dim_etat * dim_cmde];
    arm_matrix_instance_f32 *calc11= new arm_matrix_instance_f32;

    void init();

public:    
    kalman(){
        init();
    }

    arm_status recopie(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dest);

    void kalman_predict(float32_t *U);
    void kalman_predict() { kalman_predict(U_data); };
    void affiche_etat(float *compar);
    void affiche_etat(){affiche_etat(NULL);};
    void affiche_precision();
    int testprincipal();

    void kalman_maj(arm_matrix_instance_f32 *Mesure_c, arm_matrix_instance_f32 *H, arm_matrix_instance_f32 *HT, arm_matrix_instance_f32 *R);
    void kalman_maj(){kalman_maj(Mesure1_c, H1, H1T, R1); };

}; // class kalman


//fonctions .cpp
float genbruitblanc(unsigned int precision, float largeur);
float moyennebruit(int n, unsigned int precision, float largeur);
float test60bruit(int n, unsigned int precision, float largeur);
void affiche_mat(arm_matrix_instance_f32 *mat);

#endif