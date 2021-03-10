#include "kalman.h"
#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

/* but: tester le kalman 
    pour l'instant, le mieux est de compiler à la main
*/
#include "arm_math.h"
#include "math_helper.h"

void kalman::init()
    {
        //Serial1.println("initdeb");
        arm_mat_init_f32(X_c, dim_etat, 1, X_data);
        arm_mat_init_f32(U_c, dim_cmde, 1, U_data);
        arm_mat_init_f32(P_c, dim_etat, dim_etat, P_data);
        X = X_data;
        P = P_data;
        U = U_data;
        
        arm_mat_init_f32(Mesure1_c, dim_mesure1, 1, Mesure1_data);

        arm_mat_init_f32(F, dim_etat, dim_etat, (float32_t *)F_data);
        arm_mat_trans_f32(F, FT);
        arm_mat_init_f32(Q, dim_cmde, dim_cmde, (float32_t *)Q_data);
        arm_mat_init_f32(H1, dim_mesure1, dim_etat, H1_data);
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
        arm_mat_init_f32(calc8b, dim_etat, dim_mesure1, calc8b_data);
        arm_mat_init_f32(calc9, dim_mesure1, dim_mesure1, calc9_data);
        arm_mat_init_f32(calc10, dim_mesure1, dim_mesure1, calc10_data);
        arm_mat_init_f32(Id_etat, dim_etat, dim_etat, (float32_t *)Id_etat_data);
        //Serial1.println("initfin");
        
    }

arm_status kalman::recopie(arm_matrix_instance_f32 *src, arm_matrix_instance_f32 *dest){
        arm_status status;
        //#ifdef ARM_MATH_MATRIX_CHECK
        if (src->numCols!=dest->numCols || src->numRows!=dest->numRows)
          {  status = ARM_MATH_SIZE_MISMATCH;}
        else
       
        {
            float32_t *pIn = src->pData, *pOut=dest->pData;
            int i; 
            
            for (i=(src->numRows)*(src->numCols); i>=0; i--)
                *(pOut+i)=*(pIn+i);
            
            status = ARM_MATH_SUCCESS;
        }
        return status;
        
    }
void kalman::kalman_predict(float32_t *U)
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
    
    //prendre en compte l'angle
    F_data[2] = -U[0] * Dt * sin(X[2]);
    F_data[dim_etat + 2] = -U[0] * Dt * cos(X[2]);
    
    //Pkp1m = F*(Pk*FT) + Q
    statut = arm_mat_mult_f32(P_c, FT, calc3);
    statut = arm_mat_mult_f32(F, calc3, calc4);
    statut = arm_mat_add_f32(Q, calc4, P_c)//à corriger d'urgence..
    
    //débug: vérifier si statut = ARM_MATH_SUCCESS;
    }

void kalman::kalman_maj(arm_matrix_instance_f32 *Mesure_c, arm_matrix_instance_f32 *H, arm_matrix_instance_f32 *HT, arm_matrix_instance_f32 *R)
    {
    /* 
    * Met à jour X et P à partir de Mesure, pointeur vers mesure de position.
    * Mesure est un pointeur, issu indifféremment de l'odométrie ou de la caméra 
    * H : matrice d'observation, par défaut à H1 (=identité)
    */
    arm_mat_mult_f32(H, X_c, calc6);
    Serial1.println("H"); affiche_mat(H);
    Serial1.println("X"); affiche_mat(X_c);
    Serial1.println("calc6"); affiche_mat(calc6);
    
    arm_mat_sub_f32(Mesure_c, calc6, calc7); //calcul résidu
    
    //if(statut!=ARM_MATH_SUCCESS) Serial1.println("erreur calcul");
    Serial1.println("calc7"); affiche_mat(calc7);
    
    arm_mat_mult_f32(P_c, HT, calc8);  //calc8=P*Ht
    arm_mat_mult_f32(H1, calc8, calc9); //calc9=H*P*Ht

    arm_mat_add_f32(calc9, R1, calc10); //S = H*P*Ht+R1
    arm_mat_inverse_f32(calc10, calc9); //S^-1
    arm_mat_mult_f32(calc8, calc9, calc8b); //calcul gain Kalman : P*Ht*S^-1
    arm_mat_mult_f32(calc8b, calc7, calc2);//K*Ytilde : mise à jour de X
    arm_mat_add_f32(X_c, calc2, X_c);//mise à jour état. Ca marche?
    //affiche_etat();
    arm_mat_mult_f32(calc8b, H, calc3);//K*H
    arm_mat_sub_f32(Id_etat, calc3, calc4);//(I-K*H)
    arm_mat_mult_f32(calc4, P_c, calc3);//(I-K*H)*P
    recopie(calc3, P_c);
    //affiche_precision();
    }

float genbruitblanc(int precision, float largeur){
    /* bruit pseudo-aléatoire. précision max: max(unsigned int)*/
    //deux variables uniforme entre +/- précision
    float bruit = (rand()%(2*precision) - precision)*largeur/precision;
    Serial1.printf("bruit :%f ", bruit);
    return bruit;
    
}

float moyennebruit(int n, unsigned int precision, float largeur){
    int i;
    float s=0;
    for(i=0;i<n;i++)
        s+=genbruitblanc(precision, largeur)/n;
    return s;
}

float test60bruit(int n, unsigned int precision, float largeur){
    float x=0, val;
    int i;
    for (i=0; i<n; i++){
        val = (fabsf(genbruitblanc(precision, 2*largeur))<largeur) ? 100/n : 0.;
        x+=val;
        //printf("val: %f\tx: %f\n",val, x);
    }
    return x;

}

void affiche_mat(arm_matrix_instance_f32 *mat){
    int i,j;
    for(i=0; i<mat->numRows; i++){
        for(j=0; j<mat->numCols; j++)
            Serial1.printf("%f ",(mat->pData)[i*mat->numRows+j]);
        Serial1.printf("\n");
    }
}


void kalman::affiche_etat(float *compar){
    int i;
    Serial1.printf("etat: ");
    for (i=0; i<dim_etat; i++)
        Serial1.printf("%f\t", X[i]);
    Serial1.printf("\n");
    if (compar!=NULL){
        Serial1.printf("reel: ");
        for (i=0; i<dim_etat; i++)
            Serial1.printf("%f\t", compar[i]);
        Serial1.printf("\n");
    }
}

void kalman::affiche_precision(){
    int i;
    Serial1.printf("Vars: ");//variances
    for (i=0; i<dim_etat; i++)
        Serial1.printf("%f\t", P[i*dim_etat+i]);
    Serial1.printf("\n");
}

int kalman::testprincipal(){
    int temps = 10;
    float32_t dt =0.1;
    float32_t EtatReel[10][3];//x,y,theta
    int i,j;
    float32_t Commande[2] = {1,0.0};//v, omega
    
    U[0]=Commande[0];
    U[1]=Commande[1];
    X[0]=0.5;
    X[1]=-0.3;
    X[2]=0.;
    
    for(i=0;i<3;i++)
        EtatReel[0][i]=0.;

    for(i=1;i<temps;i++){
        EtatReel[i][0] = Commande[0]*dt*cos(EtatReel[i-1][2])+EtatReel[i-1][0];
        EtatReel[i][1] = Commande[0]*dt*sin(EtatReel[i-1][2])+EtatReel[i-1][1];
        EtatReel[i][2] = Commande[1]*dt+EtatReel[i-1][2];
        //Serial1.printf("etat reel %d: %f %f %f\n", i, EtatReel[i][0], EtatReel[i][1], EtatReel[i][2]);
    }
    Serial1.printf("\ntemps %d\n",0);
    affiche_etat(EtatReel[1]);
    for(i=1; i<temps; i++){
        Serial1.printf("\ntemps %d\n",i);
        for (j=0;j<3;j++){
            Mesure1_data[j]=EtatReel[i][j]+genbruitblanc(100, M1largeurbb);
            Serial1.printf("mes:%f reel:%f\n", Mesure1_data[j], EtatReel[i][j]);
        }
        kalman_predict(); //on utilise U = constante.
        //kalman_maj();     //on utilise la mesure ci-dessus. 
        affiche_etat(EtatReel[i]);
        affiche_precision();
    }
    
    return 0;
} 