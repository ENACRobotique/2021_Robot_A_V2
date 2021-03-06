//Kalman: voir namespace

/*
    AFAIRE:
    sommes-nous dans un modèle linéaire?
    supprimer inutile ci_dessous
    mettre à jour le système: F,Q,B,H..
    implémenter kalman_correct
    implémenter fonction principale, appelée par navigator ou l'inverse ? #KalmanZeBigBos

*/

#include "arm_math.h"
#include "math_helper.h"


namespace kalman{
    /* X: etat :x, y, theta, v, omega
    *  P: Cov état
    *  F: systeme linearisé
    *  B,U: mat commande et commande
    *  Q: bruit sur commande
    */

    //public:
    const uint16_t dim_etat=5, dim_cmde=1;
    float32_t *X=NULL;//état du systeme, alias X_data
    float32_t *P=NULL;//idem P_A, P_B
    float32_t *U=NULL;//commande

    //privé:
    arm_status statut;
    //à créer, ou déjà en mémoire? -> supprimer lesz allocations statiques de X,P,U
    float32_t X_data[dim_etat];//c'est possible, ça ? [dim_etat]
    arm_matrix_instance_f32 *X_c;//calculs

    float32_t U_data[dim_cmde];
    arm_matrix_instance_f32 *U_c;
    float32_t P_data[dim_etat];
    arm_matrix_instance_f32 *P_c;

    
    /* description du systeme */
    const float32_t B_data[dim_etat*dim_cmde] =
    {
    0., 0., 0., 1., 1.
    };
    arm_matrix_instance_f32 *B;
    arm_matrix_instance_f32 *BT;

    float32_t F_data[dim_etat*dim_etat] =
    {
    /* systeme linearisé */
    0., 0., 0., 0.,
    0., 0., 0., 0.,
    0., 0., 0., 0.,
    0., 0., 0., 0.,
    };
    arm_matrix_instance_f32 *F;
    arm_matrix_instance_f32 *FT;

    const float32_t Q_data[dim_cmde*dim_cmde] =
    {
        1.0
    };
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
    float32_t calc3_data[dim_etat*dim_etat];
    float32_t calc4_data[dim_etat*dim_etat];
    arm_matrix_instance_f32 *calc3;
    arm_matrix_instance_f32 *calc4;
    // taille cmde*dim_etat
    float32_t calc5_data[dim_cmde*dim_etat];
    arm_matrix_instance_f32 *calc5;

    void init(){
        arm_mat_init_f32(F, dim_etat, dim_etat, F_data);
        arm_mat_trans_f32(F, FT);
        arm_mat_init_f32(B, dim_etat, dim_cmde, (float32_t *)B_data);
        arm_mat_trans_f32(B, BT);
        arm_mat_init_f32(Q, dim_cmde, dim_cmde, (float32_t *)Q_data);
        

        arm_mat_init_f32(calc1, dim_etat, 1, calc1_data);
        arm_mat_init_f32(calc2, dim_etat, 1, calc2_data);
        arm_mat_init_f32(calc3, dim_etat, dim_etat, calc3_data);
        arm_mat_init_f32(calc4, dim_etat, dim_etat, calc4_data);
        arm_mat_init_f32(calc5, dim_cmde, dim_etat, calc5_data);

        arm_mat_init_f32(X_c, dim_etat, 1, X_data);
        arm_mat_init_f32(U_c, dim_cmde, 1, U_data);
        arm_mat_init_f32(P_c, dim_etat, dim_etat, P_data);
        
    }

    void kalman_predict(){
        //Xkp1m = F*Xk +B*U
        statut=arm_mat_mult_f32(F, X_c, calc1);
        statut=arm_mat_mult_f32(B, U_c, calc2);
        statut=arm_mat_add_f32(calc1, calc2, X_c);

        //Pkp1m = F*(Pk*FT) + B*(Q*BT)
        statut=arm_mat_mult_f32(P_c, FT, calc3);
        statut=arm_mat_mult_f32(F, calc3, calc4);
        statut=arm_mat_mult_f32(Q, BT, calc5);
        statut=arm_mat_mult_f32(B, calc5, calc3);
        statut=arm_mat_add_f32(calc3, calc4, P_c);
        //débug: vérifier si statut = ARM_MATH_SUCCESS;
    }

    void kalman_correct(){

    }

}
