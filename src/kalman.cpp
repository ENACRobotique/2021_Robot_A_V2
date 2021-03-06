#include "kalman.h"
#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

/* but: tester le kalman 
    pour l'instant, le mieux est de compiler à la main
*/




float genbruitblanc(unsigned int precision, float largeur){
    /* bruit pseudo-aléatoire. précision max: max(unsigned int)*/
    //deux variables uniforme entre +/- précision
    return (rand()%(2*precision) - precision)*largeur/precision;
    
}

float moyennebruit(int n, unsigned int precision, float largeur){
    int i;
    float s;
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

void affiche_etat(float *compar=NULL){
        using namespace kalman;
        int i;
        Serial1.printf("etat: ");
        for (i=0; i<dim_etat; i++)
            Serial1.printf("%f\t", X[i]);
        Serial1.printf("\n");
        if (compar!=NULL){
            Serial1.printf("reel: ");
            for (i=0; i<dim_etat; i++)
                Serial1.printf("%f\t", X[i]);
            Serial1.printf("\n");
        }
    }

void affiche_precision(){
    using namespace kalman;
    int i;
    Serial1.printf("Vars: ");//variances
    for (i=0; i<dim_etat; i++)
        Serial1.printf("%f\t", P[i*dim_etat+i]);
    Serial1.printf("\n");
}

int testprincipal(){
    int temps = 500;
    float dt =0.1;
    float EtatReel[500][3];//x,y,theta
    int i,j;
    float Commande[3] = {1,0.01};//v, omega
    for(i=0;i<3;i++)
        EtatReel[0][i]=0.;

    for(i=1;i<temps;i++){
        EtatReel[i][0] = Commande[0]*dt*cos(EtatReel[i-1][2]);
        EtatReel[i][1] = Commande[0]*dt*sin(EtatReel[i-1][2]);
        EtatReel[i][2] = Commande[1]*dt+EtatReel[i-1][2];
    }

    
    using namespace kalman;
    i=0;
    U[0]=Commande[0];
    U[1]=Commande[1];
    X[0]=0.5;
    X[1]=-0.3;
    X[2]=0.01;
    for(i=0; i<temps; i++){
        for (j=0;j<3;j++)
            Mesure1_data[j]=EtatReel[i][j]+genbruitblanc(100, M1largeurbb);
        kalman_predict(); //on utilise U = constante.
        kalman_maj();     //on utilise la mesure ci-dessus. 
        affiche_etat(EtatReel[i]);
        affiche_precision();
    }
    


} 