#ifndef GL_CONST_H
#define GL_CONST_H

#define CN_PI_CONSTANT 3.14159265359
#define CN_SQRT2_CONSTANT 1.41421356237

#define CN_LOG          "_log"

//Parametrs Type
#define CN_PT_LOGLVL    0   // log level
#define CN_PT_AL        1   // anglelimit
#define CN_PT_D         2   // distance
#define CN_PT_W         3   // weight
#define CN_PT_SL        4   // steplimit
#define CN_PT_CRF       5   // circle raduis factor
#define CN_PT_CHW       6   // curvature heuristic weight
#define CN_PT_DDF       7   // coefficient to decrease distance
#define CN_PT_LC        8   // linecost
#define CN_PT_DM        9   // distancemin
#define CN_PT_CLC       10  // check lesser circle
#define CN_PT_NOP       11  // number of parents to increase radius

#define CN_PT_NUM      12 // количество параметров


//Default values of parameters
#define CN_PTD_AL       30
#define CN_PTD_D        4
#define CN_PTD_W        2
#define CN_PTD_SL       20000   // Максимальное количество шагов цикла поиска,
                                // после которых будет сделан вывод, что пути не
                                // существует
#define CN_PTD_DD       100     // На сколько будем делить размер карты, чтобы получить
                                // радиус шага
#define CN_PTD_LR       1

#define CN_PTD_LC       1.0     //linecost

#define CN_PTD_DDF      1       // коэффициент, на который делится distance для получения меньшего радиуса

#define CN_PTD_DMIN     3       // минимальный допустимый радиус

#define CN_PTD_NOFPTOINCRAD 2   // количество предков, у которых должен совпадать радиус, чтобы можно было увеличить радиус теущей вершины

//Obstacle
#define CN_OBSTL 1

//loglevel
#define CN_LOGLVL_NO	0
#define CN_LOGLVL_HIGH	1
#define CN_LOGLVL_MED	1.5
#define CN_LOGLVL_LOW	2


/*
 * XML file tags ---------------------------------------------------------------
 */
#define CNS_TAG_ROOT "root"
    #define CNS_TAG_ALGORITHM       "algorithm"
    #define CNS_TAG_MAP             "map"
    #define CNS_TAG_HEIGHT          "height"
    #define CNS_TAG_WIDTH           "width"
    #define CNS_TAG_SX              "startx"
    #define CNS_TAG_SY              "starty"
    #define CNS_TAG_FX              "finishx"
    #define CNS_TAG_FY              "finishy"
    #define CNS_TAG_GRID            "grid"
        #define CNS_TAG_ROW         "row"
    #define CNS_TAG_ANGLELIMIT      "anglelimit"
    #define CNS_TAG_DISTANCE        "distance"
    #define CNS_TAG_WEIGHT          "weight"
    #define CNS_TAG_STEPLIMIT       "steplimit"
    #define CNS_TAG_CRADIUSFACTOR   "circleRadiusFactor"
    #define CNS_TAG_CURVHEURWEIGHT  "curvatureHeuristicWeight"
    #define CNS_TAG_CHECKCIRCLE     "checkLesserCircle"
    #define CNS_TAG_LINECOST        "linecost"
    #define CNS_TAG_DISTANCEMIN     "distanceMin"
    #define CNS_TAG_DECRDISTFACTOR  "decreaseDistanceFactor"
    #define CNS_TAG_NOFPTOINCRAD    "numOfParentsToIncreaseRadius"
    #define CNS_TAG_OPTIONS         "options"
    #define CNS_TAG_LOGLVL          "loglevel"
    #define CNS_TAG_LOG             "log"
        #define CNS_TAG_MAPFN       "mapfilename"
        #define CNS_TAG_SUM         "summary"
        #define CNS_TAG_PATH        "path"
        #define CNS_TAG_ROW         "row"
        #define CNS_TAG_LPLEVEL     "lplevel"
        #define CNS_TAG_HPLEVEL     "hplevel"
        #define CNS_TAG_ANGLES      "angles"
        #define CNS_TAG_ANGLE       "angle"
        #define CNS_TAG_LOWLEVEL    "lowlevel"
            #define CNS_TAG_SECTION "section"
            #define CNS_TAG_STEP    "step"
            #define CNS_TAG_OPEN    "open"
            #define CNS_TAG_NODE    "node"
            #define CNS_TAG_CLOSE   "close"

/*
 * End of XML files tags -------------------------------------------------------
 */

/*
 * XML files tag's attributes --------------------------------------------------
 */
    #define CNS_TAG_ATTR_NUMOFSTEPS     "numberofsteps"
    #define CNS_TAG_ATTR_NODESCREATED   "nodescreated"
    #define CNS_TAG_ATTR_LENGTH         "pathlength"
    #define CNS_TAG_ATTR_SECTIONS       "sections"
    #define CNS_TAG_ATTR_TIME           "time"
    #define CNS_TAG_ATTR_X              "x"
    #define CNS_TAG_ATTR_Y              "y"
    #define CNS_TAG_ATTR_NUM            "number"
    #define CNS_TAG_ATTR_F              "F"
    #define CNS_TAG_ATTR_G              "g"
    #define CNS_TAG_ATTR_PARX           "parent_x"
    #define CNS_TAG_ATTR_PARY           "parent_y"
    #define CNS_TAG_ATTR_VALUE          "value"
    #define CNS_TAG_ATTR_SX             "start.x"
    #define CNS_TAG_ATTR_SY             "start.y"
    #define CNS_TAG_ATTR_FX             "finish.x"
    #define CNS_TAG_ATTR_FY             "finish.y"
    #define CNS_TAG_ATTR_PF             "pathfound"
        #define CNS_TAG_ATTR_TRUE       "true"
        #define CNS_TAG_ATTR_FALSE      "false"
    #define CNS_TAG_ATTR_MAXANGLE       "maxAngle"



/*
 * End of XML files tag's attributes -------------------------------------------
 */
#endif
