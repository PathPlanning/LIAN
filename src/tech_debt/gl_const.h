#ifndef GL_CONST_H
#define GL_CONST_H

constexpr auto CN_EPSILON = 1e-8;

constexpr auto CN_LOG = "_log";

//Parametrs Type
constexpr auto CN_PT_LOGLVL = 0;   // log level
constexpr auto CN_PT_AL = 1;  // anglelimit
constexpr auto CN_PT_D = 2;  // distance
constexpr auto CN_PT_W = 3;  // weight
constexpr auto CN_PT_SL = 4;  // steplimit
constexpr auto CN_PT_CHW = 5;  // curvature heuristic weight
constexpr auto CN_PT_DDF = 6;  // coefficient to decrease distance
constexpr auto CN_PT_DM = 7;  // distancemin
constexpr auto CN_PT_PC = 8;  // pivot circle value
constexpr auto CN_PT_NOP = 9;  // number of parents to increase radius
constexpr auto CN_PT_PS = 10;  // post smoother

constexpr auto CN_PT_NUM = 11;  // number of parameters


//Default values of parameters

constexpr auto CN_PTD_AL = 30;
constexpr auto CN_PTD_D = 4;
constexpr auto CN_PTD_W = 2;
constexpr auto CN_PTD_SL = 20000;// Maximum number of steps, after this number the path is considered nonexistent
constexpr auto CN_PTD_LR = 1;

constexpr auto CN_PTD_DDF = 1;// Value that divides distance in order to decrease radius of the step
constexpr auto CN_PTD_DMIN = 3;// Radius minimum

constexpr auto CN_PTD_NOFPTOINCRAD = 2;// Number of parent vertecies that must have the same radius in order to start the increase of current radius

//Obstacle

constexpr auto CN_OBSTL = 1;

//loglevel

constexpr auto CN_LOGLVL_NO = 0;
constexpr auto CN_LOGLVL_TINY = 1;
constexpr auto CN_LOGLVL_HIGH = 2;
constexpr auto CN_LOGLVL_ITER = 3;
constexpr auto CN_LOGLVL_MED = 4;
constexpr auto CN_LOGLVL_LOW = 5;

/*
 * XML file tags ---------------------------------------------------------------
 */


/*
 * End of XML files tags -------------------------------------------------------
 */


 /*
  * XML files tag's attributes --------------------------------------------------
  */



  /*
   * End of XML files tag's attributes -------------------------------------------
   */

#endif
