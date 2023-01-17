#ifndef BARBOT_ACTIONS_DEFS_HPP
#define BARBOT_ACTIONS_DEFS_HPP

namespace barbot
{ 
    double jointParams_init[7] = {0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.00};
    double jointParams_ready[7] = {0.78, -1.31, -1.12, 2.20, 0.37, 1.39, -2.07};

    double gripper_open[2]={0.04, 0.04};
    double gripper_close[2]={0.03, 0.03};

    
    // x y qx qy qz qw
    double bar_loc[6]={7.05405438377, -3.05123224287, 0.0, 0.0, -0.770362096194, 0.637606650489};
    double initial_loc[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double table1_loc[6]={3.66013340332, 3.88313181018, 0.0, 0.0, 0.661400205324, 0.750033178197};
    double table2_loc[6]={0.519050181886, 3.57388980882, 0.0,0.0, 0.76838303885, 0.639990238682};
    double table3_loc[6]={-2.47429314697, 3.96756514915, 0.0, 0.0, 0.742997221484, 0.669294500849};
    double table4_loc[6]={-5.38004718078, 3.84192372261, 0.0, 0.0, 0.734146314158, 0.678991302896};

struct jointParams
{
    /* data */
    double arm_joint_1;
    double arm_joint_2;
    double arm_joint_3;
    double arm_joint_4;
    double arm_joint_5;
    double arm_joint_6;
    double arm_joint_7;
    jointParams()
    {
        // default position
        arm_joint_1=0.2;
        arm_joint_2=-1.34;
        arm_joint_3=-0.2;
        arm_joint_4=1.94;
        arm_joint_5=-1.57;
        arm_joint_6=1.37;
        arm_joint_7=0.00;
    }
    // 0.07 -0.34 -1.73 1.35 -1.91 0.28 0.00
};


struct headParams
{
    double head_joint_1;
    double head_joint_2;

    headParams()
    {
        head_joint_1=0.0;
        head_joint_2=0.0;
    }

};

struct markerPos
{
    struct position
    {
        double x;
        double y;
        double z;
    }position;
    
    struct orientation
    {
        double x;
        double y;
        double z;
        double w;
    }orientation; 
};

struct coordiParams
{
    double x;
    double y;
    double qx;
    double qy;
    double qz;
    double qw;
    coordiParams()
    {
        // startiong position 1.97, -0.56, 0.0, 0.0, 0.68, 0.72
        x=1.97;
        y=-0.56;
        qx=0.0;
        qy=0.0;
        qz=0.68;
        qw=0.72;
    }
};

    
} // namespace barbot

#endif