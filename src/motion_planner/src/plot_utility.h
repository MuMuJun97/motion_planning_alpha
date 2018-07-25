#ifndef PLOT_UTILITY_H
#define PLOT_UTILITY_H
#include "boost/tuple/tuple.hpp"
#include "gnuplot-iostream.h"// Downloads/gnuplot-iostream-master/gnuplot-iostream-master/gnuplot-iostream.h"
#include <string>
//std::ostream& operator<<(std::ostream& os, const std::string& obj)
//{
//    std::string formated = obj;
//    os << std::string obj;
//    return os;

//}
class plot_utility
{
public:
    plot_utility(){}
    /*
     * plot the 1-d data and the data will be connected piece by piece.
     */
    void plot_random_point(std::vector<std::pair<double,double> > data,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,6'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
     //   Gp1 <<
        // Gp1 << "set output "<< << "\n";
        //          Gp1 << "set xrange [-4:4]\nset yrange [0:40]\n";
        //          Gp1 << "set xtics 0.4\n";
        //          Gp1 << "set mxtics 2 \n";
        //          Gp1 << "set size ratio -1 \n";
        //          Gp1 << "set hidden3d nooffset\n";
         Gp1 << "set size ratio -1 \n";
         Gp1 << "set xtics 1\n";
        Gp1 << "plot '-' with points pt 5 ps 0.2 lc rgb 'black'\n";
        Gp1.send1d(data);
        //Gp1.send1d(boundary);

    }
    void plot_tree_reference_obstacle(std::vector<std::pair<double, double> > reference_path,
                              std::vector<std::vector<std::pair<double, double> > > treedata,
                                      std::vector<std::pair<double, double> > obstacles,
                                      std::string name)
    {
        Gnuplot Gp1;
     //   std::vector<std::pair<double,double>> line_start1,line_end1,line_start2,line_start3;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << "set xtics 1 \n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << tmp;
        Gp1 << "plot";
        Gp1 << Gp1.binFile2d(treedata,"record") << "with lines lt 73 lc rgb 'grey' lw 0.01 notitle 'vec of vec of std::pair'";
        Gp1 << ",";
        Gp1 <<  Gp1.binFile1d(reference_path,"record")<<"with lines lt 2 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 << ",";
        Gp1 <<  Gp1.binFile1d(obstacles,"record")<<"with points pt 5 ps 0.2 lc rgb 'blue' notitle 'vec of std::pair'";
        Gp1 << ",";
        Gp1 << std::endl;
    }
    void plot_environment(std::vector<std::pair<double,double> > obstacles,std::vector<std::pair<double,double> >Boundary
                          )
    {
        std::vector<std::pair<double,double> >temp;
        temp.push_back(std::make_pair(30,10));
        temp.push_back(std::make_pair(30,100));
        Gnuplot Gp1;
        Gp1 << "set terminal eps font 'arial,2'\n";
        Gp1 << "set output 'environment.png'\n";
        Gp1 << "set xrange [28:40] \n set yrange [0:110]\n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << "set object 1 rect from 30,10 to 38,100 lw 0.1\n";
        Gp1 << "set object 2 rect from 34.5,50 to 37.5,54 lw 0.1\n";
     //   Gp1 << "set object 3 rect from 33.2,18.2 to 34.8,22 lw 0.1\n";
        Gp1 << "plot '-' with points lt 2 lc rgb 'black' lw 0.5\n";
        Gp1.send1d(temp);
    }

    void plot_checked_left_right_curve(std::vector<std::pair<double,double> > realpath,
                                   std::vector<std::pair<double,double> > left_curve,
                                   std::vector<std::pair<double,double> > right_curve,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << "plot";
        Gp1 << Gp1.binFile1d(realpath,"record") << "with lines lt 2 lc rgb 'black' ps 0.1 notitle 'vec of std::pair'";
        //     Gp1 << Gp1.binFile1d(realpath,"record") << "with points pointtype 5 ps 0.01 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(left_curve,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(right_curve,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << std::endl;
    }
    /*
     * plot the 1-d data and the data will be connected piece by piece.
     */
    void plot_1d_data(std::vector<std::pair<double,double> > data,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,6'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
     //   Gp1 <<
        // Gp1 << "set output "<< << "\n";
        //          Gp1 << "set xrange [-4:4]\nset yrange [0:40]\n";
        //          Gp1 << "set xtics 0.4\n";
        //          Gp1 << "set mxtics 2 \n";
              //    Gp1 << "set size ratio -1 \n";
        //          Gp1 << "set hidden3d nooffset\n";
                 Gp1 << "set size ratio -1 \n";
        Gp1 << "plot '-' with points lt 2 lc rgb 'black' lw 0.5\n";
        Gp1.send1d(data);
        //Gp1.send1d(boundary);

    }
    void plot_obs_car(std::vector<std::pair<double,double> > obs,
                                   std::vector<std::pair<double,double> > car,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,5'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
        // Gp1 << "set output "<< << "\n";
      //  Gp1 << "set xrange [28:40]\nset yrange [0:100]\n";
        Gp1 << "set xtics 0.4\n";
        //          Gp1 << "set mxtics 2 \n";
        Gp1 << "set size ratio -1 \n";
        //          Gp1 << "set hidden3d nooffset\n";
        // Gp1 << "plot '-' with lines lt 2 lc rgb 'black' lw 0.5\n";
        //Gp1.send1d(realpath);
        //  Gp1.send1d(realpath);
        Gp1 << "plot";
        // Gp1 << Gp1.binFile1d(realpath,"record") << "with lines lt 2 lc rgb 'black' ps 1 notitle 'vec of std::pair'";
        Gp1 << Gp1.binFile1d(car,"record") << "with points pt 10 ps 0.2 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(obs,"record") << "with points pt 5 ps 0.5 notitle 'vec of std::pair'";
        Gp1 << std::endl;
    }
    void plot_reference_obstacle_locally(std::vector<std::pair<double,double> > realpath,

                                         std::vector<std::pair<double,double> > reference_path,
                                         std::vector<std::pair<double,double> > originalpath,
                                         std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,5'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
        // Gp1 << "set output "<< << "\n";
      //  Gp1 << "set xrange [28:40]\nset yrange [0:100]\n";
        Gp1 << "set xtics 1\n";
        //          Gp1 << "set mxtics 2 \n";
        Gp1 << "set size ratio -1 \n";
        //          Gp1 << "set hidden3d nooffset\n";
        // Gp1 << "plot '-' with lines lt 2 lc rgb 'black' lw 0.5\n";
        //Gp1.send1d(realpath);
        //  Gp1.send1d(realpath);
        Gp1 << "plot";
        // Gp1 << Gp1.binFile1d(realpath,"record") << "with lines lt 2 lc rgb 'black' ps 1 notitle 'vec of std::pair'";
        Gp1 << Gp1.binFile1d(realpath,"record") << "with lines notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(reference_path,"record") << "with lines lc rgb 'black' notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(originalpath,"record") << "with points pt 5 ps 0.5 notitle 'vec of std::pair'";
        Gp1 << std::endl;
    }
    void plot_solution(std::vector<std::pair<double,double> > realpath,
                           std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,5'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
        // Gp1 << "set output "<< << "\n";
      //  Gp1 << "set xrange [28:40]\nset yrange [0:100]\n";
        Gp1 << "set xtics 1\n";
        //          Gp1 << "set mxtics 2 \n";
        Gp1 << "set size ratio -1 \n";
        //          Gp1 << "set hidden3d nooffset\n";
        // Gp1 << "plot '-' with lines lt 2 lc rgb 'black' lw 0.5\n";
        //Gp1.send1d(realpath);
        //  Gp1.send1d(realpath);
        Gp1 << "plot";
        // Gp1 << Gp1.binFile1d(realpath,"record") << "with lines lt 2 lc rgb 'black' ps 1 notitle 'vec of std::pair'";
        Gp1 << Gp1.binFile1d(realpath,"record") << "with lines notitle 'vec of std::pair'";
        //Gp1 <<",";
        //Gp1 << Gp1.binFile1d(obs,"record") << "with points pt 5 ps 0.5 notitle 'vec of std::pair'";
        Gp1 << std::endl;

    }
    void plot_solution_obs(std::vector<std::pair<double,double> > realpath,
                           std::vector<std::pair<double,double> > obs,
                           std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,5'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
        // Gp1 << "set output "<< << "\n";
      //  Gp1 << "set xrange [28:40]\nset yrange [0:100]\n";
        Gp1 << "set xtics 1\n";
        //          Gp1 << "set mxtics 2 \n";
        Gp1 << "set size ratio -1 \n";
        //          Gp1 << "set hidden3d nooffset\n";
        // Gp1 << "plot '-' with lines lt 2 lc rgb 'black' lw 0.5\n";
        //Gp1.send1d(realpath);
        //  Gp1.send1d(realpath);
        Gp1 << "plot";
        Gp1 << Gp1.binFile1d(realpath,"record") << "with lines lt 2 lc rgb 'black' ps 1 notitle 'vec of std::pair'";
        //Gp1 << Gp1.binFile1d(realpath,"record") << "with lines notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(obs,"record") << "with points pt 5 ps 0.5 notitle 'vec of std::pair'";
        Gp1 << std::endl;

    }
    void plot_solutions_obs(
                           std::vector<std::pair<double,double> > obs,
                           std::vector<std::vector<std::pair<double,double> > > saved_path,
                           std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,5'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
        Gp1 << "set xtics 1\n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << "plot";
        Gp1 << Gp1.binFile1d(obs,"record") << "with points pt 5 ps 0.5 notitle 'vec of std::pair'";
         Gp1 <<",";
        std::vector<std::vector<std::pair<double,double> > > temp1;
        for (int i=0;i<saved_path.size();i++)
        {
            temp1.push_back(saved_path[i]);

            Gp1 << Gp1.binFile2d(temp1,"record") << "with lines lt 3 lc rgb 'red' lw 0.1 notitle 'vec of vec of std::pair'";
            temp1.clear();
            Gp1 <<",";
        }
        Gp1 << std::endl;
    }
    void plot_1d_data_withoriginal(std::vector<std::pair<double,double> > realpath,
                                   std::vector<std::pair<double,double> > originalpath,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,5'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
        Gp1 << "set xtics 1\n";
        //          Gp1 << "set mxtics 2 \n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << "set grid back\n";
        Gp1 << "plot";
        // Gp1 << Gp1.binFile1d(realpath,"record") << "with lines lt 2 lc rgb 'black' ps 1 notitle 'vec of std::pair'";
        Gp1 << Gp1.binFile1d(realpath,"record") << "with lines notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(originalpath,"record") << "with points 5 ps notitle 'vec of std::pair'";
        Gp1 << std::endl;
    }
    void plot_keep_lane(std::vector<std::pair<double,double> >  Reference_Path,
                         std::vector<std::pair<double,double> >  Left_Boundary,
                         std::vector<std::pair<double,double> > Right_Boundary,std::string Name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+Name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << tmp;
        Gp1 << "plot";
        Gp1 << Gp1.binFile1d(Reference_Path,"record") << "with lines lt 2 lc rgb 'black' ps 0.1 notitle 'vec of std::pair'";
        //     Gp1 << Gp1.binFile1d(realpath,"record") << "with points pointtype 5 ps 0.01 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(Left_Boundary,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(Right_Boundary,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";

        Gp1 << std::endl;
                        }
    void plot_keep_lane_boundary_polygon(std::vector<std::pair<double,double> >  Reference_Path,
                         std::vector<std::pair<double,double> >  boundary_polygon,
                        std::string Name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+Name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << tmp;
        Gp1 << "plot";
        Gp1 << Gp1.binFile1d(Reference_Path,"record") << "with lines lt 2 lc rgb 'black' ps 0.1 notitle 'vec of std::pair'";
        //     Gp1 << Gp1.binFile1d(realpath,"record") << "with points pointtype 5 ps 0.01 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(boundary_polygon,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 << std::endl;
                        }
    void plot_keep_lane_boundary_polygon_collision(
                             std::vector<std::pair<double,double> >  boundary_polygon,
                                                    std::vector<std::pair<double,double> >  car,
                            std::string Name)
        {
            Gnuplot Gp1;
            std::string tmp="set output \'"+Name+"\' \n";
            Gp1 << "set terminal eps font 'Arial,2'\n";
            Gp1 << tmp;
            Gp1 << "plot";
            Gp1 << Gp1.binFile1d(boundary_polygon,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
            Gp1 <<",";
            Gp1 << Gp1.binFile1d(car,"record") << "with lines lt 3 lc rgb 'red' lw 0.1 notitle 'vec of std::pair'";
            Gp1 << std::endl;
                            }
    /*
    * plot the 2-d data, with only one input
    */
    void plot_tree_data(std::vector<std::vector<std::pair<double,double> > > data,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << "set xtics 0.4\n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << tmp;
        Gp1 << "plot";
        Gp1 << Gp1.binFile2d(data,"record") << "with lines lt 2 lc rgb 'red' lw 0.1 notitle 'vec of vec of std::pair'";
        Gp1 << std::endl;
    }
    void plot_keep_lane_randompoint_test(std::vector<std::pair<double,double> >  Reference_Path,
                         std::vector<std::pair<double,double> >  Left_Boundary,
                         std::vector<std::pair<double,double> > Right_Boundary,
                                         std::vector<std::pair<double,double> > randompoint,std::string Name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+Name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << tmp;
        Gp1 << "plot";
        Gp1 << Gp1.binFile1d(Reference_Path,"record") << "with lines lt 2 lc rgb 'black' ps 0.1 notitle 'vec of std::pair'";
        //     Gp1 << Gp1.binFile1d(realpath,"record") << "with points pointtype 5 ps 0.01 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(Left_Boundary,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(Right_Boundary,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(randompoint,"record") <<  "with points pointtype 5 ps 0.1 notitle 'vec of std::pair'";

        Gp1 << std::endl;
                        }

    void plot_1d_data_withoriginal(std::vector<std::pair<double,double> >  path1,
                                   std::vector<std::pair<double,double> >  path2,
                                   std::vector<std::pair<double,double> > path3,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
     //   Gp1 << "set xrange [34:36]\nset yrange [0:100]\n";
        Gp1 << "set xtics 0.4\n";
        //          Gp1 << "set mxtics 2 \n";
        Gp1 << "set size ratio -1 \n";
        // Gp1 << "set output "<< << "\n";
        //          Gp1 << "set xrange [-4:4]\nset yrange [0:40]\n";
        //          Gp1 << "set xtics 0.4\n";
        //          Gp1 << "set mxtics 2 \n";
        Gp1 << "set size ratio -1 \n";
     //   Gp1 << "set object 1 rect from 30,10 to 37,100 lw 0.1\n";
       // Gp1 << "set object 2 rect from 34,50 to 36,54 lw 0.1\n";
        //          Gp1 << "set hidden3d nooffset\n";
        //          Gp1 << "plot '-' with lines lt 2 lc rgb 'black' lw 0.5\n";
        //          //   Gp1.send1d(boundary);
        //          Gp1.send1d(data);
        Gp1 << "plot";
        Gp1 << Gp1.binFile1d(path1,"record") << "with lines lt 2 lc rgb 'black' ps 0.1 notitle 'vec of std::pair'";
        //     Gp1 << Gp1.binFile1d(realpath,"record") << "with points pointtype 5 ps 0.01 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(path2,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(path3,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";

        Gp1 << std::endl;
    }

    void plot_1d_data_withoriginal(std::vector<std::pair<double,double> > realpath,
                                   std::vector<std::pair<double,double> > originalpath,std::vector<std::vector<std::pair<double,double> > > plannedtraj,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << tmp;// "set output 'dsds.png' \n";
      //  Gp1 << "set xrange [28:40]\nset yrange [0:110]\n";
        Gp1 << "set size ratio -1 \n";
     //   Gp1 << "set object 1 rect from 30,10 to 37,100 lw 0.1\n";
       // Gp1 << "set object 2 rect from 34.5,50 to 36,54 lw 0.1\n";
        Gp1 << "plot";
        Gp1 << Gp1.binFile1d(realpath,"record") << "with lines lt 2 lc rgb 'black' ps 0.1 notitle 'vec of std::pair'";
        //     Gp1 << Gp1.binFile1d(realpath,"record") << "with points pointtype 5 ps 0.01 notitle 'vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile1d(originalpath,"record") << "with lines lt 3 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
        Gp1 <<",";
        std::vector<std::vector<std::pair<double,double> > > temp1;
        for (int i=0;i<plannedtraj.size();i++)
        {
            temp1.push_back(plannedtraj[i]);

            Gp1 << Gp1.binFile2d(temp1,"record") << "with lines lt 3 lc rgb 'red' lw 0.1 notitle 'vecof vec of std::pair'";
            temp1.clear();
            Gp1 <<",";
        }
        Gp1 << std::endl;
    }
    /*
    * plot the 2-d data, with only one input
    */
    void plot_2d_data(std::vector<std::vector<std::pair<double,double> > > data,std::string name)
    {
        Gnuplot Gp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,4'\n";
      //  Gp1 << "set output '2d_data_two_inputs.png'\n";
 //       Gp1 << "set xrange [34:36]\nset yrange [10:100]\n";
        Gp1 << "set xtics 0.4\n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << "set size ratio -1 \n";
     //   Gp1 << "set xrange [28:40]\nset yrange [0:110]\n";
  //      Gp1 << "set object 1 rect from 30,10 to 38,100 lw 0.1\n";
   //     Gp1 << "set object 2 rect from 34.5,50 to 37.5,54 lw 0.1\n";
        Gp1 << tmp;
        Gp1 << "plot";
        Gp1 << Gp1.binFile2d(data,"record") << "with lines lt 2 lc rgb 'red' lw 0.1 notitle 'vec of vec of std::pair'";
        Gp1 << std::endl;
    }
    /*
    * for plot the solutions
    */
    void plot_2d_solution(std::vector<std::vector<std::pair<double,double> > > data,std::vector<std::pair<double,double> > originalpath,std::string name)
    {
        Gnuplot Gp1;
        std::vector<std::vector<std::pair<double,double> > > temp1;
        std::string tmp="set output \'"+name+"\' \n";
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << tmp;
        Gp1 << "set size ratio -1 \n";
        Gp1 << "set xrange [28:40]\nset yrange [0:110]\n";
        Gp1 << "set object 1 rect from 30,10 to 37,100 lw 0.1\n";
        Gp1 << "set object 2 rect from 33,50 to 36,54 lw 0.1\n";
        Gp1 << "plot";
        for (int i=0;i<data.size();i++)
        {
            temp1.push_back(data[i]);
            Gp1 << Gp1.binFile2d(temp1,"record") << "with lines lt 2 lc rgb 'blue' lw 0.5 notitle 'vec of vec of std::pair'";
            Gp1 <<",";
            temp1.clear();
        }
        Gp1 << Gp1.binFile1d(originalpath,"record") << "with lines lt 3 lc rgb 'red' lw 0.1 notitle 'vec of std::pair'";

        Gp1 << std::endl;
    }

    /*
    * plot the 2-d data, with two inputs
    */
    void plot_2d_data(std::vector<std::vector<std::pair<double,double> > > data1,
                      std::vector<std::vector<std::pair<double,double> > > data2)
    {
        std::vector<std::vector<std::pair<double,double> > > temp1;
        //    temp1.push_back(solutions[0]);
        //    temp1.push_back(solutions[1]);
        Gnuplot Gp1;
        Gp1 << "set terminal eps font 'Arial,2'\n";
        Gp1 << "set output '2d_data_two_inputs.png'\n";
        //      Gp1 << "set xrange [34:36]\nset yrange [0:100]\n";
        Gp1 << "set xtics 0.4\n";
        //          Gp1 << "set mxtics 2 \n";
        Gp1 << "set size ratio -1 \n";
        Gp1 << "set xrange [28:40]\nset yrange [0:110]\n";
        Gp1 << "set object 1 rect from 30,10 to 38,100 lw 0.1\n";
        Gp1 << "set object 2 rect from 34.5,50 to 37.5,54 lw 0.1\n";
        //       Gp1 << "set xtics 0.4\n";
        //       Gp1 << "set mxtics 2 \n";
        //       Gp1 << "set size ratio -1 \n";
        //       Gp1 << "set hidden3d nooffset\n";
        //       int index = 1;
        //   //    for (int i=0;i<Grids.size();i++)
        //   //    {
        //   //        Rect Re;
        //   //        Re.from[0]= Grids[i][0].X/1000.0;
        //   //        Re.from[1]= Grids[i][0].Y/1000.0;
        //   //        Re.to[0] = Grids[i][2].X/1000.0;
        //   //        Re.to[1] = Grids[i][2].Y/1000.0;
        //   //        Gp1 << "set object " << index++ <<" rectangle "<< Re <<" lw 0.1\n";
        //   //    }
        //       for (int i=0;i<GridObs.size();i++)
        //       {
        //           Rect Re;
        //           Re.from[0]= GridObs[i][0].X/1000.0;
        //           Re.from[1]= GridObs[i][0].Y/1000.0;
        //           Re.to[0] = GridObs[i][2].X/1000.0;
        //           Re.to[1] = GridObs[i][2].Y/1000.0;
        //           Gp1 << "set object " << index++ <<" rectangle "<< Re <<" fillstyle solid fc rgb 'red' front lw 0.01\n";
        //       }


        Gp1 << "plot";
        Gp1 << Gp1.binFile2d(data1,"record") << "with lines lt 2 lc rgb 'yellow' lw 1 notitle 'vec of vec of std::pair'";
        Gp1 <<",";
        Gp1 << Gp1.binFile2d(data2,"record") << "with lines lt 2 lc rgb 'blue' lw 0.2 notitle 'vec of vec of std::pair'";
        Gp1 << std::endl;
        //       for (int i=0;i<solutions.size();i++)
        //       {
        //           temp1.push_back(solutions[i]);
        //        Gp1 << Gp1.binFile2d(data2,"record") << "with lines lt 2 lc rgb 'blue' lw 1 notitle 'vec of vec of std::pair'";
        //        Gp1 <<",";
        //        temp1.clear();
        //       }
        //        Gp1 << std::endl;
    }

     void plot_tree_for_thesis(std::vector<std::pair<double,double> > reference_path,std::vector<std::pair<double,double> > reference_path1,
                               std::vector<std::vector<std::pair<double,double> > > treedata,
                               std::vector<std::pair<double,double> > sampling_nodes,
                               std::string name)
     {
         Gnuplot Gp1;
         /*std::vector<std::pair<double,double> > line_start1,line_end1,line_start2,line_start3;
         line_start1.push_back(std::make_pair(0,4+3.5/2.0+3.5));
         line_start1.push_back(std::make_pair(30,4+3.5/2.0+3.5));
         line_start3.push_back(std::make_pair(0,4+3.5/2.0));
         line_start3.push_back(std::make_pair(30,4+3.5/2.0));
         line_start2.push_back(std::make_pair(0,4-3.5/2.0));
         line_start2.push_back(std::make_pair(30,4-3.5/2.0));*/

         std::string tmp="set output \'"+name+"\' \n";
         Gp1 << "set terminal eps font 'Arial,2' background 'black'\n";
       //  Gp1 << "set output '2d_data_two_inputs.png'\n";
        // Gp1 << "set xrange [0:40]\nset yrange [:100]\n";
         Gp1 << "set xtics 0.4\n";
         Gp1 << "set size ratio -1 \n";
         Gp1 << "set size ratio -1 \n";
         Gp1 << "set object 1 rect from 25,3 to 28,5 lw 0.1 fillstyle solid fc rgb 'white' \n";
        // Gp1 << "set xrange [28:40]\nset yrange [0:110]\n";
      //   Gp1 << "set object 1 rect from 30,10 to 38,100 lw 0.1\n";
      //   Gp1 << "set object 2 rect from 34.5,50 to 37.5,54 lw 0.1\n";
         Gp1 << tmp;
         Gp1 << "plot";
         Gp1 << Gp1.binFile2d(treedata,"record") << "with lines lt 73 lc rgb 'grey' lw 0.01 notitle 'vec of vec of std::pair'";
         Gp1 << ",";
         Gp1 <<  Gp1.binFile1d(reference_path,"record")<<"with lines lt 2 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
         Gp1 << ",";
         //Gp1 <<  Gp1.binFile1d(line_start1,"record")<<"with lines lt 2 lc rgb 'yellow' lw 4 notitle 'vec of std::pair'";
         //Gp1 << ",";
         //Gp1 <<  Gp1.binFile1d(line_start3,"record")<<"with lines lt 10 dashtype 2 lc rgb 'yellow' lw 4 notitle 'vec of std::pair'";
         //Gp1 << ",";
           //Gp1 <<  Gp1.binFile1d(line_start2,"record")<<"with lines lt 2 lc rgb 'yellow' lw 4 notitle 'vec of std::pair'";
          //Gp1 << ",";
           Gp1 <<  Gp1.binFile1d(sampling_nodes,"record")<<"with points pt 7 lc rgb 'red' ps 0.1 notitle 'vec of std::pair'";
            Gp1 << ",";
       //    Gp1 <<  Gp1.binFile1d(reference_path1,"record")<<"with lines lt 2 lc rgb 'blue' lw 0.1 notitle 'vec of std::pair'";
           Gp1 << std::endl;
     }
     void plot_solutions_for_thesis(std::vector<std::pair<double,double> > reference_path,
                               std::vector<std::vector<std::pair<double,double> > > solutionsdata,std::string name)
     {
         Gnuplot Gp1;
         std::string tmp="set output \'"+name+"\' \n";
         Gp1 << "set terminal eps font 'Arial,2' background 'black'\n";
       //  Gp1 << "set output '2d_data_two_inputs.png'\n";
        // Gp1 << "set xrange [0:40]\nset yrange [:100]\n";
         Gp1 << "set xtics 0.4\n";
         Gp1 << "set size ratio -1 \n";
         Gp1 << "set size ratio -1 \n";
        // Gp1 << "set xrange [28:40]\nset yrange [0:110]\n";
      //   Gp1 << "set object 1 rect from 30,10 to 38,100 lw 0.1\n";
      //   Gp1 << "set object 2 rect from 34.5,50 to 37.5,54 lw 0.1\n";
         Gp1 << tmp;
         Gp1 << "plot";
         Gp1 << Gp1.binFile2d(solutionsdata,"record") << "with lines lt 2 lc rgb 'red' lw 0.1 notitle 'vec of vec of std::pair'";
         Gp1 << ",";
         Gp1 <<  Gp1.binFile1d(reference_path,"record")<<"with lines lt 2 lc rgb 'yellow' lw 0.1 notitle 'vec of std::pair'";

         Gp1 << std::endl;
     }
    /*
     * add more functions according to the requirements.
     */

};
#endif // PLOT_UTILITY_H
