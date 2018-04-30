#include <iostream>
#include <math.h> //do sinusa
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>//
#include <pcl/io/pcd_io.h>//wczytywanie chmury
#include <pcl/common/common.h>//do getMinMax3D
#include <pcl/io/png_io.h> // do zapisu do PNG
#include <pcl/common/eigen.h>//do uzyskania macierzy transformacji (bool pcl::transformBetween2CoordinateSystems)
#include <pcl/common/transforms.h> //do transformacji chmury ( pcl::transformPointCloud) z wykorzystaniem macierzy transformacji
#include <string>
#include <vector>

using namespace std;
#define PI 3.14159265

//(w funkcji project zrobilem przekazywanie chmury przez wkaznik, a w save przez referencje - w funkcjach standarodwych PCL stosuje sie na ogol ta druga opcje)
//h -parametr 1 - skladowa "z" wektora prostopadlego do plaszczyzny na ktora rzutujemy - okresla kat pod jakim ogladamy obiekt wzdluz jego wysokosci - mozna to potem wyrazic np. procentowo; parametr 2 - kat pod jakim ogladamy obiekt wszerz (podajemy w stopniach)
void project(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_rotated,double h,double alfa_deg)
{

    //kolejne wartosci tablicy, to kolejne paramaetry plasczyzny, na
    //ktora rzutujemy chmure 3D Ax+By+Cz+D=0
    //warto wykorzystac fakt, ze wektor [A,B,C] jest wektorem normalnym do tej plaszczyzny
    //kazdy widok powinien byc skierowany w kierunku centroidu badanego obiektu
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    double alfa=0;//zmienna pomocnicza - zamiana kata ze stopni na radiany

    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    //wspolrzedne srodka boudingbox'a, mozna ewnetualnie to pozniej zastapic srodkiem ciezkosci obiektu
    double x_c=min_pt.x+(max_pt.x-min_pt.x)/2;
    double y_c=min_pt.y+(max_pt.y-min_pt.y)/2;
    double z_c=min_pt.z+(max_pt.z-min_pt.z)/2;

    double x_s=0,y_s=0,z_s=0;//wspolrzedne poczatku ukl. wsp. (bazowego) - sensora
    //double A0=x_c-x_s;
    //double B0=y_c-y_s;
    //double C0=h;
    double D=0;
    //Obrot wektora (A0,B0,C0) o kat alfa  (- wektor A,B,C jest prostopadly do plaszczyzny Ax+By+Cz+D=0)
    alfa=alfa_deg*PI/180; //bo funkcja sin() cos() przyjmuje wartosc w radianach
    double A = (x_c-x_s)*cos(alfa)-(y_c-y_s)*sin(alfa);
    double B = (x_c-x_s)*sin(alfa)+(y_c-y_s)*cos(alfa);
    double C = h;
    coefficients->values.resize (4);
    coefficients->values[0] = A;
    coefficients->values[1] = B;
    coefficients->values[2] = C;
    coefficients->values[3] = D;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    
    //TRANSFORMACJA z XYZ do XY:
    pcl::PointXYZ min_pt_proj;
    pcl::PointXYZ max_pt_proj;
    pcl::getMinMax3D(*cloud_projected, min_pt_proj, max_pt_proj);
    pcl::PointXYZ P, P1; //P1 wyznacza wektor PP1 ktory okreskla os y' , P - punkt srodkowy chmury
    double t;//zmienna pomocnicza

    //P to punkt srodkowy chmury
    P.x=min_pt_proj.x+(max_pt_proj.x-min_pt_proj.x)/2;
    P.y=min_pt_proj.y+(max_pt_proj.y-min_pt_proj.y)/2;
    P.z=min_pt_proj.z+(max_pt_proj.z-min_pt_proj.z)/2;

    if(-A*A-B*B != 0){t=(max_pt_proj.z-P.z)/(-A*A-B*B);} //wzor wynika z zaleznosci geometrycznych w przestrzeni 3D
    else{
        cout<<"Blad - dziel. przez 0";
        }
    //A i B nie moga wiec byc rownoczesnie rowne 0 -> plaszczyzne rownolegla
    //do podloza  (na ktora robie projekcje)-> nalezy postapic inaczej
    
    //rownanie parametryczne prostej okreslajacej os y'
    P1.x=P.x+t*(A*C);
    P1.y=P.y+t*(B*C);
    P1.z=max_pt_proj.z;

    Eigen::Affine3f transformation;
    Eigen::VectorXf from_line_x, from_line_y, to_line_x, to_line_y;
    Eigen::Vector3f vector_y,vector_x, vector_z;// to sa wektroy wyznaczajace kierunek osi ukladu do ktorego transforumjemy chmure
    from_line_x.resize(6); from_line_y.resize(6);
    to_line_x.resize(6); to_line_y.resize(6);

    vector_z[0]=A;
    vector_z[1]=B;
    vector_z[2]=C;

    vector_y[0]=P1.x-P.x;
    vector_y[1]=P1.y-P.y;
    vector_y[2]=P1.z-P.z;

    vector_x=vector_y.cross(vector_z);
    //normalizacja - inaczej nie dziala funkcja transformBetween2CordinateSystems
    vector_x.normalize();
    vector_y.normalize();
    vector_z.normalize();

    to_line_x << 0, 0, 0, 1, 0, 0;
    to_line_y << 0, 0, 0, 0, 1, 0;
    from_line_x << P.x,P.y,P.z,vector_x[0],vector_x[1],vector_x[2];
    from_line_y << P.x,P.y,P.z,vector_y[0],vector_y[1],vector_y[2];


    if(pcl::transformBetween2CoordinateSystems (from_line_x, from_line_y, to_line_x, to_line_y, transformation)==0)
        cout<<"nie udalo sie pozyskac macierzy transformacji"<<endl;
    cout<<"Macierz transformacji:" <<endl<<transformation.matrix()<<endl;
    pcl::transformPointCloud(*cloud_projected, *cloud_projected_rotated, transformation.matrix()); //transformacja chmury punktow (bedacej rzutem 2D chmury 3D)
}
void save(pcl::PointCloud<pcl::PointXYZ>& cloud,int w,int k, string path) //w,k liczba wierszy i kolumn tablicy
{

    pcl::PointXYZ min_pt;
    pcl::PointXYZ max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    double s,sx,sy;
    vector<vector<int>> vec (w,vector<int>(k,0)); //inicjalizowanie wektora zerami - dla zer nie trzeba pisac 0
    s=(max_pt.x-min_pt.x)/(max_pt.y-min_pt.y);

    if(max_pt.x-min_pt.x>=max_pt.y-min_pt.y) //wyprwadzenie ponizszych wzoro w zeszycie (1)
    {
        sx=k/(max_pt.x-min_pt.x);
        sy=k/((max_pt.y-min_pt.y)*s);
        for (size_t i = 0; i < cloud.points.size (); ++i)
        {
            if((static_cast<int>(round((cloud.points[i].x-min_pt.x)*sx))>=k) || (static_cast <int>(round((cloud.points[i].y-min_pt.y)*sy))>=w) )
            {continue;}  // tu chyba lepiej rozwiazac to inaczej
            else
            {vec.at(static_cast <int>(round((cloud.points[i].y-min_pt.y)*sy))).at(static_cast <int> (round((cloud.points[i].x-min_pt.x)*sx)))=1;} //dostep przez at() jest lepszy niz przez [] bo sprawdza poprawnosc zakresu, najpierw dostep do wiersza - y potem do kolumn - x

        }
    }
    else
    {
        sy=w/(max_pt.y-min_pt.y);
        sx=(w*s)/(max_pt.x-min_pt.x);
        for (size_t i = 0; i < cloud.points.size (); ++i)
        {
            if((static_cast<int>(round((cloud.points[i].x-min_pt.x)*sx))>=k) || (static_cast <int>(round((cloud.points[i].y-min_pt.y)*sy))>=w) )
            {continue;}  // tu chyba lepiej rozwiazac to inaczej
            else
            {vec.at(static_cast <int>(round((cloud.points[i].y-min_pt.y)*sy))).at(static_cast <int> (round((cloud.points[i].x-min_pt.x)*sx)))=1;} //dostep przez at() jest lepszy niz przez [] bo sprawdza poprawnosc zakresu

        }
    }

    //zapisywanie do pliku
    ofstream output_file(path);
    for(int i=vec.size()-1; i>=0; --i) {   // dla kazdego wektora w wektorze vec -> vec.size() wskazuje na ilosc wierszy
        std::copy(vec[i].cbegin(), vec[i].cend(),
                  std::ostream_iterator<int>(output_file," "));
        output_file<<'\n';
    }
    cout<<"saved to "<< path<<endl;


}
int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_rotated (new pcl::PointCloud<pcl::PointXYZ>); //utworzona dla celow testowych, do porjekcji i obrotu mozna uzywac tej samej chmury (w cl::transformPointCloud)

    pcl::io::loadPCDFile("don_cluster_8.pcd",*cloud); //jeden z wyodrebnionych podczas segmentacji obiketow

    project(cloud,cloud_projected,cloud_projected_rotated,-5,45); //tu mozna wykorzystac tylko dwie chmury (dodatkowa chmura dla analizy wizualizacji)
    save(*cloud_projected_rotated,300,300,"./example.txt");

    //WIZUALIZACJA :
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    int v0(0);
    int v1(0);
    int v2(0);
    viewer->createViewPort(0.0,0.0,0.5,1.0,v0);//create the first view port
    viewer->createViewPort(0.3,0.0,0.6,1.0,v1);
    viewer->createViewPort(0.6,0.0,1.0,1.0,v2);
    viewer->setBackgroundColor (1, 1, 1); //biale tlo, bo punkty byly czarne - wrazie braku widocznosci punktow zmienic tlo na czarne (0,0,0)

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 0, 0);
    viewer->addPointCloud(cloud,single_color,"cloud 3D",v0);
    viewer->addPointCloud(cloud_projected_rotated,single_color,"cloud projeted",v2);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud, 0, 0, 0);
    viewer->addPointCloud(cloud_projected,single_color,"projected_rotated",v1);
    viewer->addCoordinateSystem (1.0);  //to dodaje uklad wspolrzednych w punkcie (0,0,0)
    //viewer->addCoordinateSystem (1.0,1.0,1.0,1.0,"modifiedsystem",v1);

    //zapisywanie chmury
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud_projected_rotated);
    std::cerr << "Saved " << cloud_projected_rotated->points.size () << " data points to test_pcd.pcd." << std::endl;

    while (!viewer->wasStopped () )
    {
        viewer->spinOnce (100);
    }

    return (0);
}
