# LIAN

LIAN (from "limited angle") &mdash; heuristic search algorithm for generating smooth paths for single-shot grid-based 2D path finding.

## Description
Current project provides an implementation of [LIAN](https://arxiv.org/pdf/1506.01864.pdf) algorithm adapted for single-shot grid-based 2D environment.

The algorithm relies on the main idea o PathPlanning algorithms (as A\*, Theta\* and others), but designed for solving angle constrained path planning problem which is formulated as following:

Given two distinct traversable cells (star and goal), and angle limitation (value between 0 and 180) find a path such that angles of all turns are constrained with given limitation.

Besides angle limitation we also want out path to be stable and do not have turns after every meter. For this reason there is a parameter &mdash; delta, which describes the desirable length of every step. But the necessity to initialize fixed delta could cause dead-end situation near huge obstacles. In order to solve the problem: there are more parameters which define minimal value of the step and also the rule of increasing/decreasing this value.


Algorithm supports XML files as input and output format. Input file contains map and environment representation (see __"Input and Output files"__ or [samples](https://github.com/PathPlanning/LIAN/tree/master/examples))

## Getting Started

To go and try this algorithm you can use CMake.
`CMakeLists` file is available in the repository.

Notice, that project uses C++20 standard. Make sure that your compiler supports it.

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

**[CMake](https://cmake.org/)** &mdash; an open-source, cross-platform family of tools designed to build, test and package software.

### Installing

Download current repository to your local machine. Use
```
git clone https://github.com/PathPlanning/LIAN.git
```
or direct downloading.

Built current project using **Qt Creator** or **CMake**. To launch the compiled file you will need to pass input XML file as an argument. Output file for this project will be placed in the same folder as input file and, by default, will be named `_log.xml`. For examlpe, using CMake
```bash
cd PATH_TO_THE_PROJECT
cmake .
make
./LianSearch initial_file_name.xml
```
Output file for this project will be placed in the same folder as input file and, by default, will be named `_log.xml`. For examlpe,
```
"initial_file_name.xml" -> "initial_file_name_log.xml"
```
For more detailed information there are some samples in the [samples](https://github.com/PathPlanning/LIAN/tree/master/examples) folder.

## Input and Output files

Both files are an XML file with a specific structure.

Note that all tags in XML files are case sensitive. The names of tags follow some king of logic: if it is no more than 2 words &mdash; all in lowercase, if more than two words &mdash; lower camelCase.

Input file should contain:

* Mandatory tag `<map>`. It describes the environment.
    * `<height>` and `<width>` &mdash; mandatory tags that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, (*width* - 1, *height* - 1) is lower right.
    * `<startx>` and `<starty>` &mdash; mandatory tags that define horizontal (X) and vertical (Y) offset of the start location from the upper left corner. Legal values for *startx* are [0, .., *width* - 1], for *starty* - [0, .., *height* - 1].
    * `<finishx>` and `<finishy>` &mdash; mandatory tags that horizontal (X) and vertical (Y) offset of the goal location.
    * `<grid>` &mdash; mandatory tag that describes the square grid constituting the map. It consists of `<row>` tags. Each `<row>` contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" &mdash; for not traversable (actually any other figure but "0" can be used instead of "1").
    * `<cellsize>` &mdash; optional tag that defines the size of one cell. One might add it to calculate scaled length of the path.
    * `<title>`, `<URL>`, `<coordinates>`, etc &mdash; optional tags containing additional information on the map.

* Mandatory tag `<algorithm>`. It describes the parameters of the algorithm.

    * `<hweight>` &mdash; defines the weight of heuristic function. Default value is "2".
    * `<anglelimit>` &mdash; mandatory tag. Defines the maximal turn angle in trajectory.
    * `<distance>` &mdash; mandatory tag. Defines length of path section. If using eLIAN (by setting tags `<distancemin>` and `<decreaseDistanceFactor>`) defines the maximal length of path section.
    * `<distancemin>` &mdash; optional tag for using in eLIAN. Defines minimal path section length for eLIAN algorithm.
    * `<decreaseDistanceFactor>` &mdash; optional tag for using in eLIAN. Defines the factor, on which eLIAN tries to decrease path section length during search. It creates list of possible distances, started from *distance*, decreasing each time by *decreaseDistanceFactor* and finished by *distancemin*.
    * `<pivotCircleRadius>` &mdash; optional tag. If value > 0 algorithm checks the "safety zone" (circle with defined radius) around every turn point for obstacles. If there are obstacles in safety zone, there is a risk that agent during maneuver will collide with obstacle.
    * `<steplimit>` &mdash; optional tag. Set the maximal number of steps, that algorithm is allowed to make in order to prevent enormous amount of iterations, that can appear with complicated maps.
    * `<postsmoothing>` &mdash; optional tag. Defines whether the path will be smoothened (join small fluctuations in one line (if possible)) after it was found or not. Possible values &mdash; "true", "false". Default value is "false".
    * `<curvatureHeuristicWeight>` &mdash; optional tag. Defines the weight of curvature heuristic &mdash; heuristic that pays attention to the fluctuations of the path and chooses (if possible) the most straight one.
    * `<numOfParentsToIncreaseRadius>` &mdash; optional tag. Set the number of parent vertices that must have the same radius in order to start the increase of current radius. Default value is 2.

* Optional tag `<options>`. Options that are not related to search.

    * `<loglevel>` &mdash; defines the level of detalization of log-file. Default value is "1". Possible values:
        - "0" or "none" &mdash; log-file is not created.
        - "2" or "short" &mdash; All the input data is copied to the log-file plus short `<summary>` is appended. `<summary>` contains info of the path length, number of steps, elapsed time, etc.
        - "3" or "high" &mdash; 0.5 - log plus `<path>` is appended. It looks like `<grid>` but cells forming the path are marked by "\*" instead of "0". The following tags are also appended: `<hplevel>` and `<lplevel>`. `<lplevel>` is the sequence of coordinates of cells forming the path. `<hplevel>` is the sequence of sections forming the path.
        - "4" or "medium" &mdash; 1 - log plus the information (explicit enumeration) on last iteration's OPEN and CLOSE lists.
        - "5" or "full" &mdash; 1 - log plus OPEN and CLOSE lists are written into the log-file after each step of the algorithm. Can make log-files really huge and also slow down the whole algorithm.

        In order to understand differences between output files with different `<loglevel>` tags there are examples of each in [samples](https://github.com/PathPlanning/LIAN/tree/master/examples) folder.

    * `<logpath>` - defines the directory where the log-file should be written. If not specified directory of the input file is used.
    * `<logname>` - defines the name of log-file. If not specified the name of the log file is: "input file name" + "\_log" + input file extension.

The main tag in Output file, which contains path length, memory and time:
```xml
<summary pathfound="true" numberofsteps="107" nodescreated="127" length="15.414213" length_scaled="41.618375587463383" time="0.000512" sections="4" max_angle="15.9034" accum_angle="365.5412" />
```
* _"pathfound"_ &mdash; true if path was found and false otherwise
* _"numberofsteps"_ stands for the number of iterations (number of expanded vertices)
* _"nodescreated"_  stands for the number of nodes that were examined in general (= memory)
* _"length"_ stands for length of the final path
* *"length\_scaled"* stands for actual length counting the size of cell in map
* _"time"_ stands for elapsed time
* _"sections"_ stands for the number of straight sections in the paths (defines the number of turns)
* *"max\_angle"* stands for maximum turning angle value
* *"accum\_angle"* stands for accumulated turning angles value &mdash; shows total amount of fluctuations during the path
