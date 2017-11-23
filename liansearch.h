#ifndef LIANSEARCH_H
#define LIANSEARCH_H

#include "sNode.h"
#include "cList.h"
#include "cMap.h"
#include "cSearch.h"
#include <vector>
#include <unordered_map>
class LianSearch : public cSearch
{

public:

    // Конструктор с параметрами:
    LianSearch(float angleLimit, int distance, float weight,
               unsigned int steplimit, float circleRadiusFactor, float curvatureHeuristicWeight,
               float decreaseDistanceFactor, int distanceMin,
               float linecost, bool lesserCircle, int numOfParentsToIncreaseRadius);

    ~LianSearch();

    // Собственно алгоритм поиска
    SearchResult startSearch(cLogger *Log, const cMap &Map);

private:

    // Максимальный угол отклонения
    float angleLimit;

    // Минимальная дистанция шага
    int distance;

    int numOfParentsToIncreaseRadius;

    std::vector<int> listOfDistances;
    int listOfDistancesSize;

    // Вес эвристики
    float weight;

    // Другой эвристический коэффициент
    // Если проверяется местонахождение целевой точки относительно окружности
    // минимального радиуса, квадрат радиуса домножается на этот коэффициент
    float circleRadiusFactor;

    // Еще один эвристический коэффициент
    // Если используется эвристика, характеризующая отклонение траектории от прямой
    // на каждом шаге, то вычисляемая величина умножается на этот коэффициент
    float curvatureHeuristicWeight;

    float linecost; // "стоимость" движения по вертикали или горизонтали

    bool lesserCircle; // проверять ближайшие вершины на проходимость

    // максимальное число шагов цикла поиска
    unsigned int stepLimit;

    // число вершин в списках open и close
    unsigned int closeSize, openSize;

    // во сколько раз можно уменьшать изначальную длину шага
    float decreaseDistanceFactor;
    int distanceMin;

    // Виртуальные узлы, составляющие окружность
    std::vector< std::vector<Node> > circleNodes;

    std::vector<float> angles;

    // Спиок Open + итоговый путь
    cList *open, hppath, lppath;

    // список Close
    std::unordered_multimap<int, Node> close;

    void addOpen(Node &newNode);

    // метод, вычисляющий окружность по Брезенхему и записывающий
    // координаты узлов в список circleNodes (центр в точке [0, 0] )
    // радиус - радиус окружности в клетках
    void calculateCircle(int radius);

    // метод вычисляет предпочтительный радиус исходя из парамтеров карты
    int calculatePreferableRadius(const cMap &Map);

    void calculateDistances();

    Node findMin(int size);

    // метод строит отрезок с помощью алгоритма Брезенхема
    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal);

    // метод строит отрезок с помощью алгоритма Брезенхема
    // и проверяет его на наличие препятствий
    bool checkLineSegment(const cMap &Map, const Node &start, const Node &goal);

    // метод, вычисляющий "малую" окружность по брезенхему, для проверки свободного
    // пространства в опорных точках
    bool checkLesserCircle(const cMap &Map, const Node &center, const float radius);

    double calculateDistanceFromCellToCell(int start_i, int start_j, int fin_i, int fin_j);

    // критерий остановки. Возвращает истину, если цикл поиска
    // следует прекратить. Входящее значение - текущий номер шага алгоритма
    bool stopCriterion();

    int tryToIncreaseRadius(Node curNode);
    bool tryToDecreaseRadius(Node &curNode, int width);
    void findSuccessors(const Node curNode,std::vector<Node> &successors, const cMap &Map);
    bool expand(const Node curNode, const cMap &Map);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath(Node curNode);
    double makeAngles(Node curNode);
};

#endif // LIANSEARCH_H
