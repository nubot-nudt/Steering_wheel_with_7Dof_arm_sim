#include <ros/ros.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <eigen3/Eigen/Dense>

#include "std_msgs/Float64MultiArray.h"
#include "nubot_msgs/angle_position.h"
#include "nubot_msgs/simPub.h"
#include "nubot_msgs/simSub.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

#define JOINT_MODE 1
#define END_MODE 2

using namespace std;
using namespace Eigen;
using namespace chrono;

float sgn(float n)
{
    if (n > 0)
        return 1;
    else if (n < 0)
        return -1;
    else
        return 0;
}

class nubot_arm_node2
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    nubot_arm_node2()
    {

        ros::NodeHandle n;
        /*********节点*********/
        arm_param_init();
        controller = n.createTimer(ros::Duration(1.0 / 100.0), &nubot_arm_node2::timeController, this, false);
        firstCobraPubTosim_ = n.advertise<nubot_msgs::simSub>("/simSub_l", 1, true);
        firstend_pos_d_ = n.advertise<std_msgs::Float64MultiArray>("/end_pos_d_l", 1, true);
        firstend_pos_fb_ = n.advertise<std_msgs::Float64MultiArray>("/end_pos_fb_l", 1, true);

        firstCobraSubFromsim_ = n.subscribe("/simPub_l", 1, &nubot_arm_node2::firstCobraCallback, this);
        firstforce_SubFromsim_ = n.subscribe("/joint_force_pub_l", 1, &nubot_arm_node2::firstfroceCallback, this);
        firstvel_SubFromsim_ = n.subscribe("/joint_vel_pub_l", 1, &nubot_arm_node2::firstvelCallback, this);
        firstendforce_SubFromsim_ = n.subscribe("/end_force_pub_l", 1, &nubot_arm_node2::firstendforceCallback, this);
        firstendtorque_SubFromsim_ = n.subscribe("/end_torque_pub_l", 1, &nubot_arm_node2::firstendtorqueCallback, this);
        firstCobraPubTosimMsg.firstNeedMode = need_v;
    }

    ~nubot_arm_node2() // 关闭节点
    {
        std::cout << "Node closing！ Begin initializing" << std::endl;
    }
    void firstCobraCallback(const nubot_msgs::simPub &firstCobra)
    {

        for (int i = 0; i < N; i++) // 仿真使用
        {
            pos[i] = firstCobra.firstOriginalPos[i];
            theta[i] = double(M_PI * pos[i] * DH_sign[i] / 180); // 角度转换为弧度
            if (flag_init)                                       // online后仅做一次的初始化
            {
                pos_init[i] = pos[i];
                joint_4 = pos[3]; // 4电机的初始角度值
                left_or_right = 2;
                endOrJointMode = END_MODE;
                handOrBaseMode = true; // 基座坐标系
            }
        }
        flag_init = false;
        enableOK = firstCobra.enableOK;
        // time_fl = 20; //计时器
    }
    void firstfroceCallback(const std_msgs::Float64MultiArray &firstFroce)
    {
        for (int i = 0; i < N; i++) // 仿真使用
        {
            joint_force[i] = firstFroce.data[i];
        }
    }
    void firstvelCallback(const std_msgs::Float64MultiArray &firstVel)
    {
        for (int i = 0; i < N; i++) // 仿真使用
        {
            joint_vel[i] = firstVel.data[i];
        }
        // 需统一单位为rad/s
    }

    void firstendforceCallback(const std_msgs::Float64MultiArray &firstendForce)
    {
        for (int i = 0; i < 3; i++) // 仿真使用
        {
            end_force[i] = firstendForce.data[i];
        }
    }

    void firstendtorqueCallback(const std_msgs::Float64MultiArray &firstendTorque)
    {
        end_force[3] = firstendTorque.data[0];
        end_force[4] = firstendTorque.data[1];
        end_force[5] = firstendTorque.data[2];
    }
    void cacul_RHS()
    {
        Matrix<Matrix4d, 1, Dynamic> A__, J__;
        A__.resize(N);
        J__.resize(N);
        for (int i = 0; i < N; i++)
        {
            A__(i) = Matrix4d::Identity(); // 初始化A0~A6为4*4单位矩阵
            J__(i) = Matrix4d::Identity(); // 初始化A0~A6为4*4单位矩阵
        }

        // 求RHS与jacob 先Z后X
        const float er = 0.0000001; // 三角函数计算精度
        for (int i = 0; i < N; i++)
        {
            A__(i) << round(cos(theta(i) + DH_bias(i)) / er) * er, round(-cos(DH_alpha(i)) * sin(theta(i) + DH_bias(i)) / er) * er, round(sin(theta(i) + DH_bias(i)) * sin(DH_alpha(i)) / er) * er, DH_a(i) * round(cos(theta(i) + DH_bias(i)) / er) * er,
                round(sin(theta(i) + DH_bias(i)) / er) * er, round(cos(theta(i) + DH_bias(i)) * cos(DH_alpha(i)) / er) * er, round(-cos(theta(i) + DH_bias(i)) * sin(DH_alpha(i)) / er) * er, DH_a(i) * round(sin(theta(i) + DH_bias(i)) / er) * er,
                0, round(sin(DH_alpha(i)) / er) * er, round(cos(DH_alpha(i)) / er) * er, DH_d(i),
                0, 0, 0, 1;
        }

        for (int i = 0; i < N; i++)
        {
            for (int j = N - 1; j >= i; j--)
            {
                J__(i) = A__(j) * J__(i);
            }
        }
        RHS = J__(0);
        local_position << RHS(0, 3), RHS(1, 3), RHS(2, 3), RHS(3, 3);
        local_position = guodu_ * local_position; // 转换到 以上为x，右为y，前为z的基座标系，同时，以x，y各加上圆周运动的半径为目标点，进行速度方向切换
    }

    void CartesianToJoint(bool mode)
    {
        Matrix4d gg_; // 过渡矩阵
        Matrix<Matrix4d, 1, Dynamic> A__, J__;
        A__.resize(N);
        J__.resize(N);
        for (int i = 0; i < N; i++)
        {
            A__(i) = Matrix4d::Identity(); // 初始化A0~A6为4*4单位矩阵
            J__(i) = Matrix4d::Identity(); // 初始化A0~A6为4*4单位矩阵
        }

        // 求RHS与jacob 先Z后X
        const float er = 0.0000001; // 三角函数计算精度
        for (int i = 0; i < N; i++)
        {
            A__(i) << round(cos(theta(i) + DH_bias(i)) / er) * er, round(-cos(DH_alpha(i)) * sin(theta(i) + DH_bias(i)) / er) * er, round(sin(theta(i) + DH_bias(i)) * sin(DH_alpha(i)) / er) * er, DH_a(i) * round(cos(theta(i) + DH_bias(i)) / er) * er,
                round(sin(theta(i) + DH_bias(i)) / er) * er, round(cos(theta(i) + DH_bias(i)) * cos(DH_alpha(i)) / er) * er, round(-cos(theta(i) + DH_bias(i)) * sin(DH_alpha(i)) / er) * er, DH_a(i) * round(sin(theta(i) + DH_bias(i)) / er) * er,
                0, round(sin(DH_alpha(i)) / er) * er, round(cos(DH_alpha(i)) / er) * er, DH_d(i),
                0, 0, 0, 1;
        }

        for (int i = 0; i < N; i++)
        {
            for (int j = N - 1; j >= i; j--)
            {
                J__(i) = A__(j) * J__(i);
            }
        }
        RHS = J__(0);
        local_position << RHS(0, 3), RHS(1, 3), RHS(2, 3), RHS(3, 3);
        local_position = guodu_ * local_position; // 转换到 以上为x，右为y，前为z的基座标系，同时，以x，y各加上圆周运动的半径为目标点，进行速度方向切换

        for (int i = 0; i < N; i++)
        {
            Jacob(0, i) = -J__(i)(0, 0) * J__(i)(1, 3) + J__(i)(1, 0) * J__(i)(0, 3); //-nx*py+ny*px
            Jacob(1, i) = -J__(i)(0, 1) * J__(i)(1, 3) + J__(i)(1, 1) * J__(i)(0, 3); //-ox*py+oy*px
            Jacob(2, i) = -J__(i)(0, 2) * J__(i)(1, 3) + J__(i)(1, 2) * J__(i)(0, 3); //-ax*py+ay*px
            Jacob(3, i) = J__(i)(2, 0);                                               // nz
            Jacob(4, i) = J__(i)(2, 1);                                               // oz
            Jacob(5, i) = J__(i)(2, 2);                                               // az
        }
        MatrixXd CC__ = Jacob * Jacob.transpose() + MatrixXd::Identity(6, 6);
        // cout << "Jacob * Jacob.transpose() is" << endl
        //      << Jacob * Jacob.transpose() << endl;
        // cout << "CC__ is" << endl
        //      << CC__ << endl;
        // cout << "Jacob 奇异鲁棒性逆 is" << endl;
        // cout << Jacob.transpose() * pinv_eigen_based(CC__) << endl; // 奇异鲁棒性逆

        FbEndmove = Jacob * joint_vel;
        // for (int i = 0; i < 6; i++) // 处理反馈数据，接近0的要置零
        // {
        //     if (abs(FbEndmove[i]) < 1e-8)
        //     {                     // 判断阈值条件
        //         FbEndmove[i] = 0; // 将该元素置零
        //     }
        // }
        end_move_desire << dx, dy, dz, rx, ry, rz;
        if (mode)
        {
            gg_.setZero();
            gg_ = guodu_ * RHS;
            // end_move_desire = basemove_to_end(gg_, end_move_desire);
            Vector4d cacul_;
            cacul_ << end_move_desire(0), end_move_desire(1), end_move_desire(2), 1;
            cacul_ = cacul_.transpose() * gg_;
            for (int i = 0; i < 3; i++)
                end_move_desire[i] = cacul_[i];
            // cacul_ << end_move_desire(3), end_move_desire(4), end_move_desire(5), 1;
            // cacul_ = cacul_.transpose() * gg_;
            // end_move_desire[3] = cacul_[0];
            // end_move_desire[4] = cacul_[1];
            // end_move_desire[5] = cacul_[2];
            // end_move_desire[3] = end_move_desire[3] + rx_goal_end;
            // 此处可外挂一个末端系下的角运动输入接口，eg：end_move_desire[3]、end_move_desire[4]、end_move_desire[5]对应末端系下的rx，ry，rz

            //  dtheta = Jacob.transpose() * pinv_eigen_based(CC__) * end_move_desire;//奇异鲁棒逆用于保证位置跟踪speed_resolution0.2，rotate_resolution0.2
            dtheta = pinv_eigen_based(Jacob) * end_move_desire; // 伪逆用于末端速度计算
        }
        else
        {
            // dtheta = Jacob.transpose() * pinv_eigen_based(CC__) * end_move_desire;//奇异鲁棒逆用于保证位置跟踪speed_resolution0.2，rotate_resolution0.2
            dtheta = pinv_eigen_based(Jacob) * end_move_desire; // 伪逆用于末端速度计算
        }
        // cout << "dtheta is" << endl
        //      << dtheta << endl;
    }

    // 利用Eigen库，采用SVD分解的方法求解矩阵逆或者伪逆，默认误差er为0
    Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd &origin, const float err = 0.0001)
    {
        // 进行svd分解
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // 构建SVD分解结果
        Eigen::MatrixXd U = svd_holder.matrixU();
        Eigen::MatrixXd V = svd_holder.matrixV();
        Eigen::MatrixXd D = svd_holder.singularValues();
        // cout<<"D :\n"<<D<<endl;
        //  构建S矩阵
        Eigen::MatrixXd S(V.cols(), U.cols());
        S.setZero();

        for (unsigned int i = 0; i < D.size(); ++i)
        {
            if (D(i, 0) > err)
            {
                S(i, i) = 1 / D(i, 0);
            }
            else
            {
                S(i, i) = 0;
            }
        }
        // pinv_matrix = V * S * U^T
        return V * S * U.transpose();
    }

    // 将相对于基座的运动通过算子转化到末端坐标系下的运动(关系：末端运动算子=RHS*基座运动算子*RHS^-1)，输入RHS与end_move，输出一个相同的向量并赋值给end_move
    VectorXd basemove_to_end(Matrix4d &rhs, VectorXd &base_move)
    {
        Matrix4d CC_;
        CC_.setZero();
        CC_ << 0, -base_move(5), base_move(4), base_move(0),
            base_move(5), 0, -base_move(3), base_move(1),
            -base_move(4), base_move(3), 0, base_move(2),
            0, 0, 0, 0;
        CC_ = rhs * CC_ * rhs.inverse();
        VectorXd AA_;
        AA_.setZero(6);
        AA_ << CC_(0, 3), CC_(1, 3), CC_(2, 3), CC_(2, 1), CC_(0, 2), CC_(1, 0);

        return AA_;
    }

    void Target_control()
    {
        float er = 0.03;
        float Rota_speed = 0.2;
        float R_ = 0.23;

        dx = 0;
        dy = 0;
        dz = 0;
        rx = 0;
        ry = 0;
        rz = 0;

        std::cout << "相对于身体前方基坐标系坐标X:" << local_position(0) << ";Y:" << local_position(1) << ";Z:" << local_position(2) << ";" << std::endl;

        if (start_x < 1.9)
        // 圆周运动拧阀门
        {
            if (abs(Waitcount) > 1.57)
            {
                flag_end_move = !flag_end_move;
                Waitcount = 0;
                start_x++;
            }
            if (flag_end_move)
            {
                rx_goal = rotate_resolution * end_mode_speed_level * Rota_speed;
                dy_goal = speed_resolution * 5 * end_mode_speed_level * (Rota_speed * R_) * sin(Waitcount);
                dx_goal = speed_resolution * 5 * end_mode_speed_level * (Rota_speed * R_) * cos(Waitcount);
            }
            else
            {
                rx_goal = rotate_resolution * end_mode_speed_level * -Rota_speed;
                dy_goal = speed_resolution * 5 * end_mode_speed_level * (-Rota_speed * R_) * cos(abs(Waitcount));
                dx_goal = speed_resolution * 5 * end_mode_speed_level * (-Rota_speed * R_) * sin(abs(Waitcount));
            }
            Waitcount = Waitcount + FbEndmove[3] * 0.02;
            // Waitcount = Waitcount + rx_goal * 0.02;
        }
        else
        {
            dx_goal = 0;
            dy_goal = 0;
            rx_goal = 0;
        }

        std::cout << "Waitcount:  " << Waitcount << ";   " << std::endl;

        firstend_pos_d.data[0] = local_position[0] + dx_goal * 0.02;
        firstend_pos_d.data[1] = local_position[1] + dy_goal * 0.02;
        firstend_pos_d.data[2] = local_position[2] + dz_goal * 0.02;

        firstend_pos_fb.data[0] = local_position[0];
        firstend_pos_fb.data[1] = local_position[1];
        firstend_pos_fb.data[2] = local_position[2];

        firstend_pos_d_.publish(firstend_pos_d);
        firstend_pos_fb_.publish(firstend_pos_fb);
        // dx = dx_goal;
        // dy = dy_goal;
        // dz = dz_goal;
        // rx = rx_goal;
        // ry = ry_goal;
        // rz = rz_goal;
    }

    void end_force_control() // 末端阻抗控制+姿态闭环
    {
        // 导纳控制参数
        double M_f[6] = {1, 1, 1, 1, 1, 1};             // 惯性参数
        double B_f[6] = {0.1, 0.1, 0.1, 0.5, 0.5, 0.5}; // 阻尼参数
        double K_f[6] = {100, 100, 100, 200, 200, 200}; // 刚度参数
        if(joint_4>145)
        {
            M_f[2]=M_f[2]+0.1*(pos[3]-joint_4);
        }
        if(joint_4>115)
        {
            B_f[2]=B_f[2]+0.1*(pos[3]-joint_4);
        }
        if(joint_4>80 && joint_4<=115)
        {
                K_f[2]=K_f[2]+10*(pos[3]-joint_4);
        }
        if(joint_4>115 && joint_4<=145)
        {
                K_f[2]=K_f[2]+50*(pos[3]-joint_4);
        }
        if(joint_4>145)
        {
                K_f[2]=K_f[2]+(pos[3]-joint_4)^3;
        }
        // 误差参数
        VectorXd perror = VectorXd::Zero(6);
        double ferror[6] = {0};
        double last_perror[6] = {0}, last_ferror[6] = {0};

        // 临时存储
        double D_[6] = {0};
        double D_goal[6] = {dx_goal, dy_goal, dz_goal, rx_goal, ry_goal, rz_goal};

        double limited_end = 0.3;
        double limited_joint = 1.201;
        double control_time = 0.02;
        float er = 0.0001; // 三角函数计算精度

        for (int i = 0; i < 6; i++) // 力信号滤去极小噪声
            if (abs(end_force[i]) < er)
                end_force[i] = 0;


        cacul_RHS(); // 首次进入循环时需要计算一次当前位置local_position

        if (pos_desire_save)
        {
            for (int i = 0; i < 3; i++)
                end_pos_desire[i] = local_position[i];
            for (int i = 0; i < N; i++)
                joint_pos_desire[i] = pos[i]; // 用于关节角度闭环
            // attitude_desire = (guodu_ * RHS).topLeftCorner<3, 3>();
            attitude_desire.setZero();
            attitude_err.setZero();
        }
        else
        {
            std::cout << "开始跟踪: \n " << std::endl;
            for (int i = 0; i < 6; i++)
                last_perror[i] = perror[i];
            end_pos_desire[0] = D_goal[0] * control_time + end_pos_desire[0];   // 如果有跟踪速度的情况下，预测的目标位置  在ros控制消息频率为100Hz，仿真步长为10ms时，控制时间大约在0.02s
            end_pos_desire[1] = D_goal[1] * control_time + end_pos_desire[1];   // 如果有跟踪速度的情况下，预测的目标位置
            end_pos_desire[2] = D_goal[2] * control_time + end_pos_desire[2];   // 如果有跟踪速度的情况下，预测的目标位置
            attitude_desire[0] = D_goal[3] * control_time + attitude_desire[0]; // 如果有跟踪速度的情况下，预测的目标姿态
            attitude_desire[1] = D_goal[4] * control_time + attitude_desire[1]; // 如果有跟踪速度的情况下，预测的目标姿态
            attitude_desire[2] = D_goal[5] * control_time + attitude_desire[2]; // 如果有跟踪速度的情况下，预测的目标姿态
            attitude_err[0] = FbEndmove[3] * control_time + attitude_err[0];    // 累计误差
            attitude_err[1] = FbEndmove[4] * control_time + attitude_err[1];
            attitude_err[2] = FbEndmove[5] * control_time + attitude_err[2];

            perror[0] = end_pos_desire[0] - local_position[0];
            perror[1] = end_pos_desire[1] - local_position[1];
            perror[2] = end_pos_desire[2] - local_position[2];
            perror[3] = attitude_desire[0] - attitude_err[0];
            perror[4] = attitude_desire[1] - attitude_err[1];
            perror[5] = attitude_desire[2] - attitude_err[2];
            for (int i = 0; i < 6; i++)
            {
                last_ferror[i] = ferror[i];
                ferror[i] = end_force_desire[i] + end_force[i] + perror[i] * K_f[i] + (perror[i] - last_perror[i]) / control_time * B_f[i];
            }
        }
        for (int i = 0; i < 6; i++)
        {
            D_[i] = D_goal[i] + (ferror[i] + last_ferror[i]) / 2 / M_f[i] * control_time;
        }

        dx = D_[0];
        dy = D_[1];
        dz = D_[2];
        rx = D_[3];
        ry = D_[4];
        rz = D_[5];

        // std::cout << "end_pos_desire-local_position:\n"
        //           << end_pos_desire[0] - local_position[0] << ";   " << end_pos_desire[1] - local_position[1] << ";   " << end_pos_desire[2] - local_position[2] << std::endl;
        std::cout << "坐标误差X:" << perror[0] << ";Y:" << perror[1] << ";Z:" << perror[2] << ";" << std::endl;
        std::cout << "attitude_desire-attitude_err:\n"
                  << attitude_desire - attitude_err << ";   " << std::endl;


        std::cout << setw(4) << "dx:" << dx << ";dy:" << dy << ";dz:" << dz << ";rx:" << rx << ";ry:" << ry << ";rz:" << rz << ";" << std::endl;

        // std::this_thread::sleep_for(std::chrono::seconds(5)); // 延时5秒为电机归位预留时间
        if (abs(dx) > limited_end)
            dx = limited_end * sgn(dx);
        if (abs(dy) > limited_end)
            dy = limited_end * sgn(dy);
        if (abs(dz) > limited_end)
            dz = limited_end * sgn(dz);
        if (abs(rx) > limited_end)
            rx = limited_end * sgn(rx);
        if (abs(ry) > limited_end)
            ry = limited_end * sgn(ry);
        if (abs(rz) > limited_end)
            rz = limited_end * sgn(rz);

        CartesianToJoint(handOrBaseMode); // 解算速度

        if (end_force_desire.array().abs().maxCoeff() > 0.01 || end_force.array().abs().maxCoeff() > 0.01) //  || joint_force.maxCoeff() > 0.01 || joint_force.minCoeff() < -0.01
        {
            pos_desire_save = false;
        }
        else
        {
            if (perror.isZero(0.01)) // 自定义误差0.01
                pos_desire_save = true;
        }

        for (int i = 0; i < N; i++)
        {
            if (abs(dtheta(i)) > limited_joint)
            {
                std::cout << "dtheta[" << i << "]:" << dtheta(i) << "!!!!!!!!!" << std::endl;
                dtheta.setZero();
            }
        }

    }
    /***************时间中断，控制部分***************/
    void timeController(const ros::TimerEvent &e)
    {
        auto start = system_clock::now();
        if (!enableOK)
            cout << "\033[47;31m电机未初始化好!!!请等待！！！\033[0m" << endl
                 << "\033[47;31m很久没好则为网络（网线）未连接好or机械臂未上电！！！\033[0m" << endl;
        else
        {
            ros::NodeHandle n;
            n.param<int>("cobraNum1", N, 7);
            // 机械臂参数
            n.param<double>("kSpeedResolution", speed_resolution, 0.02);
            n.param<double>("kRotateResolution", rotate_resolution, 0.1);
            /*************算出机械臂各关节信息并输出到屏幕********/
            std::cout << "\033[47;31m左臂\033[0m各电机角度，单位：度以及底层电机控制量速度，单位：rpm:" << std::endl;
            for (int i = 0; i < N; i++)
            {
                std::cout << "电机[" << i + 1 << "] Angle:" << fixed << setprecision(2) << setiosflags(ios::left) << setw(7) << pos[i] << "[" << setw(4) << LimitMin[i] << "~" << setw(3) << LimitMax[i] << "];";
                std::cout << "速度控制指令: " << setiosflags(ios::left) << setw(6) << firstCobraPubTosimMsg.firstSpeed[i] << ";";
                if (endOrJointMode == JOINT_MODE)
                {
                    std::cout << "flag[" << i << "]:" << flag[i] << ";" << std::endl;
                }
                else if (endOrJointMode == END_MODE)
                {
                    std::cout << "dtheta" << i << "：" << dtheta[i] << ";" << std::endl;
                }
            }
            std::cout << std::endl;
            cout << "JointSpeedLevel:" << cobra_Vel_level << endl;
            cout << "\33[32mENDSpeedLevel:\033[0m" << end_mode_speed_level << "\33[32m   XYZ_speed:\033[0m" << speed_resolution * end_mode_speed_level << "\33[32m   RPY_speed:\033[0m" << rotate_resolution * end_mode_speed_level << endl;

            cout << endl;
            if (left_or_right == 2 || left_or_right == 1)
            {
                if (endOrJointMode == END_MODE) // 末端控制
                {

                    Target_control();
                    // if (flag_d_theta_clear) // 把各电机速度清零
                    // {
                    //     dtheta.setZero();
                    // }

                    end_force_control();

                    // CartesianToJoint(handOrBaseMode); // 解算速度
                    if (handOrBaseMode) // 显示出末端坐标系手掌or基座
                        std::cout << "\33[32m基座\033[0m坐标系控制：";
                    else
                        std::cout << "\33[32m手掌\033[0m坐标系控制：";
                    std::cout << std::endl;

                    std::cout << setw(4) << "FBdx:" << FbEndmove[0] << ";FBdy:" << FbEndmove[1] << ";FBdz:" << FbEndmove[2] << ";FBrx:" << FbEndmove[3] << ";FBry:" << FbEndmove[4] << ";FBrz:" << FbEndmove[5] << ";" << std::endl;
                    // std::this_thread::sleep_for(std::chrono::seconds(5)); // 延时5秒为电机归位预留时间
                    for (int i = 0; i < N; i++)
                        firstCobraPubTosimMsg.firstSpeed[i] = DH_sign[i] * dtheta[i]; // 仿真环境下需要修改单位为弧度每秒
                    // firstCobraPubTosimMsg.firstSpeed[i] = DH_sign[i] * dtheta[i + 1] * Motor_Reduction_Ratio[i] * 60.0 / (3.1415926 * 2);
                    firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
                }
            }
            if (endOrJointMode == JOINT_MODE)
                cout << "\33[32m关节控制\033[0m" << endl;
            if (endOrJointMode == END_MODE)
                cout << "\33[32m末端控制\033[0m" << endl;
            if (left_or_right == 1)
                cout << "\33[32m双臂控制\033[0m" << endl;
            if (left_or_right == 2)
                cout << "\33[32m左臂控制\033[0m" << endl;
            if (left_or_right == 3)
                cout << "\33[32m右臂控制\033[0m" << endl;
        }

        auto end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        cout << "control loop spent" << setprecision(7) << double(duration.count()) * microseconds::period::num / microseconds::period::den << "second" << endl;
        for (int i = 0; i <= 30; i++) // 清屏
        {
            printf(cursup);
            printf(cursclean);
        }
    }

    void arm_param_init() // 变量初始化 *(a.m_storage.m_data)@9 在调试中查看矩阵或者向量中的值
    {
        Jacob = MatrixXd::Zero(6, N);
        RHS.setZero();
        guodu_.setZero();
        // 以上为x，右为y，前为z的基座标系（机器人身体坐标系正前方），左臂1坐标系K1_l为B*trans(y，-d_)*rot（x，45）=K1_l,右臂1坐标系K1_r为B*trans(y，d_)*rot（x，-45）=K1_r
        guodu_ << 1, 0, 0, 0, 0, cos(M_PI / 4), -sin(M_PI / 4), -0.2547, 0, sin(M_PI / 4), cos(M_PI / 4), 0, 0, 0, 0, 1; // 过渡矩阵为trans(y，-d_)*rot（x，45）d_=0.2547 为机器人身体坐标系与左肩关节坐标系在y轴的距离
        local_position.setZero(4);

        // 机械臂参数调节
        cobra_Vel.resize(N);
        cobra_Vel << 167, 167, 167, 167, 167, 167, 167; // 各底层电机基础速度   ,单位：每分钟多少转  具体到机械臂角度计算： cobra_Vel/60 * 360/Motor_Reduction_Ratio = 6 * cobra_Vel/Motor_Reduction_Ratio 度/秒
        Motor_Reduction_Ratio.resize(N);
        Motor_Reduction_Ratio << 36, 36, 36, 36, 36, 36, 36; // 各电机减速比
        LimitMax.resize(N);
        LimitMax << 180, 10, 30, -5, 160, 100, 38; // 各电机最大限位 单位：° 度N
        LimitMin.resize(N);
        LimitMin << -180, -160, -150, -120, -160, -18, -105; // 各电机最小限位 单位：° 度
        temperature = VectorXd::Zero(N);                     // 电机的温度
        DianLiu = VectorXd::Zero(N);                         // 电机的电流

        firstSpeed = VectorXd::Zero(N);   // 速度指令
        original_pos = VectorXd::Zero(N); // 底层电机位置 单位：转
        pos = VectorXd::Zero(N);          // 各电机位置信息 单位：°
        pos_init = VectorXd::Zero(N);
        pos_extend = VectorXd::Zero(N);
        theta = VectorXd::Zero(N);            // 末端控制状态电机角度。单位弧度。
        dtheta = VectorXd::Zero(N);           // 末端控制状态解算的电机控制速度。单位弧度/s。
        end_move_desire = VectorXd::Zero(6);  // 末端运动的6个自由度
        joint_pos_desire = VectorXd::Zero(N); // 关节电机的期望位置

        Is_ChangeV.resize(N);
        Is_ChangeV << 1, 1, 1, 1, 1, 1, 1; // 各电机是否受加减速影响，1为可以加减速，0为屏蔽加减速。
        DH_sign.resize(N);
        // DH_sign << -1, 1, 1, 1, 1, 1, 1; // 电机摆放对DH建模正方向的影响
        // DH_sign << 1, -1, 1, 1, 1, -1, 1; // 仿真环境下自带7Dof电机摆放对DH建模正方向的影响
        DH_sign << 1, 1, 1, 1, 1, 1, 1; // 仿真环境下cc_arm_left电机摆放对DH建模正方向的影响
        cobra_motor_sign.resize(N);
        cobra_motor_sign << 1, 1, 1, 1, 1, 1, 1; // 电机与遥控器正反转适配

        // DH参数输入
        DH_a.resize(N);
        DH_alpha.resize(N);
        DH_d.resize(N);
        DH_bias.resize(N); // theta的偏置
        DH_a << 0, 0, 0, 0, 0, 0, 0.057;
        DH_alpha << 90, -90, 90, -90, -90, -90, 0;
        // DH_d << 0, 0, 0.274, 0, 0.2555, 0, 0;
        // DH_d << 0, 0, 0.292, 0, 0.323, 0, 0; // 仿真环境下自带7Dof
        DH_d << 0, 0, 0.2738, 0, 0.2555, 0, 0; // 仿真环境下cc_arm
        DH_bias << 0, 90, 90, 0, 90, -90, 0.1;
        DH_alpha = DH_alpha.array() * M_PI / 180; // 弧度化,Eigen::Array类重载了+ ， - ，* ,/ 运算符，可以直接用这些运算符对Array对象进行操作。相乘操作是对应的数字相乘，相除是对应的元素相除。
        DH_bias = DH_bias.array() * M_PI / 180;

        // 力控参数输入
        joint_force = VectorXd::Zero(N);
        joint_vel = VectorXd::Zero(N);          // 各个电机反馈的转动速度 弧度/s
        joint_force_desire = VectorXd::Zero(N); // 关节期望力
        end_force = VectorXd::Zero(6);
        FbEndmove = VectorXd::Zero(6);
        end_force_desire = VectorXd::Zero(6);
        attitude_desire.setZero();
        attitude_err.setZero();

        firstend_pos_d.data.resize(3);
        firstend_pos_fb.data.resize(3);

        end_force_desire << 0, 0, 2, 0, 0, 0;
        // joint_force_desire << 0, 0,0, 0, 0, 0, 0;
    }

private:
    ros::Subscriber firstCobraSubFromsim_;
    ros::Subscriber firstforce_SubFromsim_;
    ros::Subscriber firstvel_SubFromsim_;
    ros::Subscriber firstendforce_SubFromsim_;
    ros::Subscriber firstendtorque_SubFromsim_;

    ros::Publisher firstCobraPubTosim_;
    ros::Publisher firstend_pos_d_;
    ros::Publisher firstend_pos_fb_;

    ros::Timer controller;

    std_msgs::Float64MultiArray firstend_pos_d;
    std_msgs::Float64MultiArray firstend_pos_fb;
    nubot_msgs::simSub firstCobraPubTosimMsg;

    // 机械臂参数调节
    int N = 7; // CAN上电机个数，可设置为常量值const

    Matrix4d RHS;    // 用于计算末端位置
    Matrix4d guodu_; // 过渡矩阵
    MatrixXd Jacob;

    VectorXd cobra_Vel;
    VectorXd Motor_Reduction_Ratio;
    VectorXd LimitMax;
    VectorXd LimitMin;
    VectorXd temperature;
    VectorXd DianLiu;

    VectorXd firstSpeed;
    VectorXd original_pos;
    VectorXd pos;
    VectorXd pos_init;
    VectorXd pos_extend;
    VectorXd theta;
    VectorXd dtheta;
    VectorXd end_move_desire;
    VectorXd joint_pos_desire;

    VectorXd Is_ChangeV;
    VectorXd DH_sign;
    VectorXd cobra_motor_sign;

    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
    double dx_goal = 0, dy_goal = 0, dz_goal = 0, rx_goal = 0, ry_goal = 0, rz_goal = 0;
    double rx_goal_end = 0, ry_goal_end = 0, rz_goal_end = 0;

    int endV = 50;                 // 末端三个关节在一键模式的速度
    int cobra_Vel_Total_level = 3; // 机械臂末端运动速度档位数， 成倍增加，即3档是1档的3倍速，2档是1档的2倍速
    int end_mode_Total_level = 3;

    // DH参数输入
    VectorXd DH_a;
    VectorXd DH_alpha;
    VectorXd DH_d;
    VectorXd DH_bias; // theta的偏置

    // 导纳控制参数
    VectorXd joint_force_desire;
    VectorXd joint_force;
    VectorXd end_force;
    VectorXd joint_vel;
    VectorXd FbEndmove;
    VectorXd local_position;  // 当前末端坐标位置 (列向量)
    Vector3d attitude_desire; // 末端坐标系期望姿态  当前姿态为RHS.topLeftCorner<3,3>();
    Vector3d attitude_err;    // 旋转偏差矩阵
    double end_pos_desire[3] = {0};
    VectorXd end_force_desire;
    bool pos_desire_save = true;

    // 各种标志位...
    double joint_4;
    bool now_control = true;
    bool cobra = true;
    bool flag_init = true;
    bool flag_end_move = true;
    int start_x = 0, start_y;
    int goal_x, goal_y;

    float Waitcount = 0;
    int left_or_right = 1; // 左右臂控制切换，1为双臂，2为左臂，3为右臂
    int NOWMODE = 0;
    float VMODE = 1;
    float PMODE = 2;
    bool enableOK = false;
    int need_v = 1;                 // 标志位，程序中不应改变
    int need_p = 2;                 // 标志位，
    double rotate_resolution = 0.2; // 末端旋转速度的分辨率（越大速度越快）
    double speed_resolution = 0.05; // 末端平移速度的分辨率（越大速度越快）

    float firstNeedMode = 0;
    float firstNowMode = 0;
    int flagX = 0, flagY = 0, flagA = 0, flagB = 0, flagAB = 0, flagAX = 0, flagXY = 0, flagBY = 0; // 八种位置模式组合键
    double flag[7] = {0};                                                                           // 普通遥控各电机遥控指令
    double flag2[6] = {0};                                                                          // 末端遥控各电机指令  dx , dy , dz, rx , ry , rz;

    float flag_button = 0;
    int cobra_Vel_level = 1; // 机械臂单机速度档位
    int end_mode_speed_level = 2;

    int flag_extend = 0;         // 一键展开
    int flag_need_set_ModeV = 0; // 需要设置成速度模式
    int flag_need_set_ModeP = 0; // 需要设置成位置模式
    int flag_back = 0;           // 一键归位

    int flag_end_stop = 0;           // 末端控制停止标志符
    int endOrJointMode = JOINT_MODE; // 控制模式，1为一般控制，2为末端控制
    int flag_pos_now = 0;            // 现在正在位置控制  注意：凡是用到位置模式都需要将该标志位改为1；
    bool All_Motor_IsOnlineAndEnable = true;
    int flag_d_theta_clear = 1;
    bool handOrBaseMode = false;
};

int main(int argc, char *argv[])
{
    /*******************************ROS*****************************/
    ros::init(argc, argv, "r7_auto_sim");

    nubot_arm_node2 arm;

    ros::NodeHandle n;

    ros::spin();

    return 0;
}