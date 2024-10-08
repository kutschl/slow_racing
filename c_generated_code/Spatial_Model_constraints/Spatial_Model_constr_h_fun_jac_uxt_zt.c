/* This file was automatically generated by CasADi 3.6.6.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) Spatial_Model_constr_h_fun_jac_uxt_zt_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s4[6] = {8, 1, 0, 2, 5, 7};
static const casadi_int casadi_s5[3] = {1, 0, 0};

/* Spatial_Model_constr_h_fun_jac_uxt_zt:(i0[6],i1[2],i2[],i3[])->(o0,o1[8x1,2nz],o2[1x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *rr, *ss;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, *w9=w+9, *w10=w+11, w11;
  /* #0: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #1: @1 = 0.8 */
  w1 = 8.0000000000000004e-01;
  /* #2: @1 = (@0/@1) */
  w1  = (w0/w1);
  /* #3: @2 = 0.8 */
  w2 = 8.0000000000000004e-01;
  /* #4: @3 = input[0][5] */
  w3 = arg[0] ? arg[0][5] : 0;
  /* #5: @4 = tan(@3) */
  w4 = tan( w3 );
  /* #6: @4 = (@2*@4) */
  w4  = (w2*w4);
  /* #7: @5 = atan(@4) */
  w5 = atan( w4 );
  /* #8: @6 = sin(@5) */
  w6 = sin( w5 );
  /* #9: @7 = (@1*@6) */
  w7  = (w1*w6);
  /* #10: @8 = (@0*@7) */
  w8  = (w0*w7);
  /* #11: output[0][0] = @8 */
  if (res[0]) res[0][0] = w8;
  /* #12: @9 = zeros(8x1,2nz) */
  casadi_clear(w9, 2);
  /* #13: @10 = ones(8x1,7nz) */
  casadi_fill(w10, 7, 1.);
  /* #14: {NULL, NULL, NULL, NULL, NULL, @8, NULL, NULL} = vertsplit(@10) */
  w8 = w10[5];
  /* #15: @7 = (@7*@8) */
  w7 *= w8;
  /* #16: @11 = 1.25 */
  w11 = 1.2500000000000000e+00;
  /* #17: @11 = (@11*@8) */
  w11 *= w8;
  /* #18: @6 = (@6*@11) */
  w6 *= w11;
  /* #19: @6 = (@0*@6) */
  w6  = (w0*w6);
  /* #20: @7 = (@7+@6) */
  w7 += w6;
  /* #21: (@9[0] = @7) */
  for (rr=w9+0, ss=(&w7); rr!=w9+1; rr+=1) *rr = *ss++;
  /* #22: @5 = cos(@5) */
  w5 = cos( w5 );
  /* #23: @7 = ones(8x1,1nz) */
  w7 = 1.;
  /* #24: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, @6} = vertsplit(@7) */
  w6 = w7;
  /* #25: @3 = cos(@3) */
  w3 = cos( w3 );
  /* #26: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #27: @6 = (@6/@3) */
  w6 /= w3;
  /* #28: @2 = (@2*@6) */
  w2 *= w6;
  /* #29: @6 = 1 */
  w6 = 1.;
  /* #30: @4 = sq(@4) */
  w4 = casadi_sq( w4 );
  /* #31: @6 = (@6+@4) */
  w6 += w4;
  /* #32: @2 = (@2/@6) */
  w2 /= w6;
  /* #33: @5 = (@5*@2) */
  w5 *= w2;
  /* #34: @1 = (@1*@5) */
  w1 *= w5;
  /* #35: @0 = (@0*@1) */
  w0 *= w1;
  /* #36: (@9[1] = @0) */
  for (rr=w9+1, ss=(&w0); rr!=w9+2; rr+=1) *rr = *ss++;
  /* #37: output[1][0] = @9 */
  casadi_copy(w9, 2, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Spatial_Model_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Spatial_Model_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Spatial_Model_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Spatial_Model_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Spatial_Model_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Spatial_Model_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Spatial_Model_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void Spatial_Model_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Spatial_Model_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int Spatial_Model_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real Spatial_Model_constr_h_fun_jac_uxt_zt_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Spatial_Model_constr_h_fun_jac_uxt_zt_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Spatial_Model_constr_h_fun_jac_uxt_zt_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Spatial_Model_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Spatial_Model_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Spatial_Model_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 11;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 19;
  return 0;
}

CASADI_SYMBOL_EXPORT int Spatial_Model_constr_h_fun_jac_uxt_zt_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 11*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 19*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
