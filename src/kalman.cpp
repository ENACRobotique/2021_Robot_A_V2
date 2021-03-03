#include "arm_math.h"


//Kalman: voir namespace

/*
    AFAIRE:
    supprimer inutile ci_dessous
    kalman_correct
    fonction principale, appelée par navigator?
*/

/* ----------------------------------------------------------------------
* Copyright (C) 2010-2012 ARM Limited. All rights reserved.
*
* $Date:         17. January 2013
* $Revision:     V1.4.0
*
* Project:       CMSIS DSP Library
* Title:         arm_matrix_example_f32.c
*
* Description:   Example code demonstrating least square fit to data
*                using matrix functions:
*                    The linear combination of parameters considered is as follows: 
*                    A * X = B, where X is the unknown value and can be estimated from A & B. 
*                    The least squares estimate X is given by the following equation: 
*                    X = Inverse(AT * A) * AT * B
*
* Target Processor: Cortex-M4/Cortex-M3
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */
#include "arm_math.h"
#include "math_helper.h"
#if defined(SEMIHOSTING)
#include <stdio.h>
#endif
#define SNR_THRESHOLD   90
/* --------------------------------------------------------------------------------
* Test input data(Cycles) taken from FIR Q15 module for differant cases of blockSize
* and tapSize
* --------------------------------------------------------------------------------- */
const float32_t B_f32[4] =
{
  782.0, 7577.0, 470.0, 4505.0
};
/* --------------------------------------------------------------------------------
* Formula to fit is  C1 + C2 * numTaps + C3 * blockSize + C4 * numTaps * blockSize
* -------------------------------------------------------------------------------- */
const float32_t A_f32[16] =
{
  /* Const,   numTaps,   blockSize,   numTaps*blockSize */
  1.0,     32.0,      4.0,     128.0,
  1.0,     32.0,     64.0,    2048.0,
  1.0,     16.0,      4.0,      64.0,
  1.0,     16.0,     64.0,    1024.0,
};
/* ----------------------------------------------------------------------
* Temporary buffers  for storing intermediate values
* ------------------------------------------------------------------- */
/* Transpose of A Buffer */
float32_t AT_f32[16];
/* (Transpose of A * A) Buffer */
float32_t ATMA_f32[16];
/* Inverse(Transpose of A * A)  Buffer */
float32_t ATMAI_f32[16];
/* Test Output Buffer */
float32_t X_f32[4];
/* ----------------------------------------------------------------------
* Reference ouput buffer C1, C2, C3 and C4 taken from MATLAB
* ------------------------------------------------------------------- */
const float32_t xRef_f32[4] = {73.0, 8.0, 21.25, 2.875};
float32_t snr;
/* ----------------------------------------------------------------------
* Max magnitude FFT Bin test
* ------------------------------------------------------------------- */
int32_t main2()
{
  arm_matrix_instance_f32 A;      /* Matrix A Instance */
  arm_matrix_instance_f32 AT;     /* Matrix AT(A transpose) instance */
  arm_matrix_instance_f32 ATMA;   /* Matrix ATMA( AT multiply with A) instance */
  arm_matrix_instance_f32 ATMAI;  /* Matrix ATMAI(Inverse of ATMA) instance */
  arm_matrix_instance_f32 B;      /* Matrix B instance */
  arm_matrix_instance_f32 X;      /* Matrix X(Unknown Matrix) instance */
  uint32_t srcRows, srcColumns;  /* Temporary variables */
  arm_status status;
  /* Initialise A Matrix Instance with numRows, numCols and data array(A_f32) */
  srcRows = 4;
  srcColumns = 4;
  arm_mat_init_f32(&A, srcRows, srcColumns, (float32_t *)A_f32);
  /* Initialise Matrix Instance AT with numRows, numCols and data array(AT_f32) */
  srcRows = 4;
  srcColumns = 4;
  arm_mat_init_f32(&AT, srcRows, srcColumns, AT_f32);
  /* calculation of A transpose */
  status = arm_mat_trans_f32(&A, &AT);
  /* Initialise ATMA Matrix Instance with numRows, numCols and data array(ATMA_f32) */
  srcRows = 4;
  srcColumns = 4;
  arm_mat_init_f32(&ATMA, srcRows, srcColumns, ATMA_f32);
  /* calculation of AT Multiply with A */
  status = arm_mat_mult_f32(&AT, &A, &ATMA);
  /* Initialise ATMAI Matrix Instance with numRows, numCols and data array(ATMAI_f32) */
  srcRows = 4;
  srcColumns = 4;
  arm_mat_init_f32(&ATMAI, srcRows, srcColumns, ATMAI_f32);
  /* calculation of Inverse((Transpose(A) * A) */
  status = arm_mat_inverse_f32(&ATMA, &ATMAI);
  /* calculation of (Inverse((Transpose(A) * A)) *  Transpose(A)) */
  status = arm_mat_mult_f32(&ATMAI, &AT, &ATMA);
  /* Initialise B Matrix Instance with numRows, numCols and data array(B_f32) */
  srcRows = 4;
  srcColumns = 1;
  arm_mat_init_f32(&B, srcRows, srcColumns, (float32_t *)B_f32);
  /* Initialise X Matrix Instance with numRows, numCols and data array(X_f32) */
  srcRows = 4;
  srcColumns = 1;
  arm_mat_init_f32(&X, srcRows, srcColumns, X_f32);
  /* calculation ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B) */
  status = arm_mat_mult_f32(&ATMA, &B, &X);
  /* Comparison of reference with test output */
  snr = arm_snr_f32((float32_t *)xRef_f32, X_f32, 4);
  /*------------------------------------------------------------------------------
  *            Initialise status depending on SNR calculations
  *------------------------------------------------------------------------------*/
  status = (snr < SNR_THRESHOLD) ? ARM_MATH_TEST_FAILURE : ARM_MATH_SUCCESS;
  
  if (status != ARM_MATH_SUCCESS)
  {
#if defined (SEMIHOSTING)
    printf("FAILURE\n");
#else
    while (1);                             /* main function does not return */
#endif
  }
  else
  {
#if defined (SEMIHOSTING)
    printf("SUCCESS\n");
#else
    while (1);                             /* main function does not return */
#endif
  }
  return 0;
}


























namespace kalman{
    /* X: etat
    *  P: Cov état
    *  F: systeme linearisé
    *  B,U: mat commande et commande
    *  Q: bruit sur commande
    */

    //public:
    const uint16_t dim_etat=4, dim_cmde=1;
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

    
    int idbuff;

    /* description du systeme */
    const float32_t B_data[dim_etat*dim_cmde] =
    {
    782.0, 7577.0, 470.0, 4505.0
    };
    arm_matrix_instance_f32 *B;
    arm_matrix_instance_f32 *BT;

    const float32_t F_data[dim_etat*dim_etat] =
    {
    /* systeme linearisé */
    1.0,     32.0,      4.0,     128.0,
    1.0,     32.0,     64.0,    2048.0,
    1.0,     16.0,      4.0,      64.0,
    1.0,     16.0,     64.0,    1024.0,
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
        arm_mat_init_f32(F, dim_etat, dim_etat, (float32_t *)F_data);
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

}
