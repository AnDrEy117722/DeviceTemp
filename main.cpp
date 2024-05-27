#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <fstream>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

// Структура для хранения точек данных
struct Point {
    double t, Z;
};

void RANSAC(const std::vector<Point>& points, double& bestA, double& bestB, int maxIterations = 20, double threshold = 2);



int main(int, char**)
{

    ifstream fdata;

    vector<Point> points;
    Point data_point;

    if (!fdata.is_open())
        fdata.open("data.txt");
    while (!fdata.eof())
    {
        std::string Z, T;
        getline(fdata, Z, ' ');
        getline(fdata, T);
        data_point.Z = stod(Z);
        data_point.t = stod(T);
        points.push_back(data_point);
    }

    double bestA = 0, bestB = 0;
    RANSAC(points, bestA, bestB);

    cout << "a: " << bestA << " b: " << bestB << endl;

    for (size_t i = 0; i < points.size(); i++)
        cout << "Calculated T: " << (log10(points[i].Z) - bestA)/bestB << " Data T: " << points[i].t << endl;
    return 0;
}

/*
Z = 1024*(R0*10^(K(t-T0)))/(Rc+R0) =
    1024*(10^(K(t-T0)))/(Rc/R0+1) =
    1024*(10^(K(t-T0)))/(A+1)

1024*(10^-T0K)/(A+1) = const = C;
Z = C*10^(Kt)
lg(Z) = lg(C) + tK
lg(Z) = C1 + tK
Estimated parameters C1 and K;
*/

// Функция для вычисления параметров A и B линейной модели на основе двух точек
void fitModel(const Point& p1, const Point& p2, double& A, double& B) {
    double t1 = p1.t, t2 = p2.t;
    double logZ1 = std::log10(p1.Z), logZ2 = std::log10(p2.Z);

    if (t1 == t2) {
        throw std::invalid_argument("Points have the same t value");
    }

    B = (logZ2 - logZ1) / (t2 - t1);
    A = logZ1 - B * t1;
}

// Функция для вычисления ошибки точки относительно модели
double computeError(const Point& p, double A, double B) {
    double predictedLogZ = A + B * p.t;
    double actualLogZ = std::log10(p.Z);
    return std::abs(predictedLogZ - actualLogZ);
}

// RANSAC алгоритм
void RANSAC(const std::vector<Point>& points, double& bestA, double& bestB, int maxIterations, double threshold) {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> distr(0, points.size()-1);

    int bestInlierCount = -1;

    for (int iter = 0; iter < maxIterations; ++iter) {
        // Выбираем случайные две точки
        int i1 = distr(gen);
        int i2 = distr(gen);
        Point p1 = points[i1];
        Point p2 = points[i2];

        while (p1.t == p2.t)
            p2 = points[distr(gen)];

        double A, B;
        try {
            fitModel(p1, p2, A, B);
        } catch (const std::exception& e) {
            continue; // Если не удалось вычислить модель, переходим к следующей итерации
        }

        // Подсчитываем количество inliers
        int inlierCount = 0;
        for (const auto& point : points) {
            double error = computeError(point, A, B);
            if (error < threshold) {
                ++inlierCount;
            }
        }

        // Обновляем лучшую модель, если нашли большее количество inliers
        if (inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestA = A;
            bestB = B;
        }
    }
}
