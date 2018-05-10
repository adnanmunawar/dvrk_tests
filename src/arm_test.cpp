#include <ros/ros.h>
#include <dvrk_arm/Arm.h>
#include <math.h>
#include <iomanip>

int main(int argc, char **argv){
    DVRK_Arm arm("MTMR");
    int sleep_us = 10;
    int exp_itrs = 20;
    if (argc > 1){
        sleep_us = atoi(argv[1]);
        std::cout << "Sleeping for " << sleep_us <<" (micro s) between each callback" << std::endl;
    }
    if (argc > 2){
        exp_itrs = atoi(argv[2]);
        std::cout << "Using 1 x exp(" << exp_itrs <<") callback iterations" << std::endl;
    }
    int cnt = 1<<exp_itrs;
    double x,y,z;
    double px,py,pz;
    double diff, diff2, bandwidth;
    ros::init(argc, argv, "dvrk_thread_safety_test");
    ros::Time time;
    double t_offset = time.now().toSec();
    for (int i = 0; i < cnt; i++){
        px = x ; py = y ; pz = z;
        arm.measured_cp_pos(x,y,z);
        diff = (px - x) + (py - y) + (pz - z);
        diff2 = sqrt(diff);
        if (diff2 > 0.00001){
            std::cout << std::endl << "Error Geater than 0.00001: " << diff << ": " << diff2 << std::endl;
        }
        double percent = ((double) i / (double)cnt) * 100.0;
        if (i % 1000 == 0){
            double time_sec = (ros::Time::now().toSec() - t_offset);
            bandwidth = i / time_sec;
            std::cout << '\r'
                      <<"Time:" << std::setw(10) << std::setfill(' ') << time_sec
                      <<"| Percent Done: " << std::setw(10) << std::setfill(' ') << percent
                      <<"| Bandwidth: "  << std::setw(10) << std::setfill(' ') << bandwidth << "(Cb/s)"
                      <<std::setw(5) << std::setfill(' ') <<  std::flush;
        }
        usleep(sleep_us);
    }
    return 0;
}
