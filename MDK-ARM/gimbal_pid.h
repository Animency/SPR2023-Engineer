/**************************************/
 /******      ID使用说明：        ******/
 /******   0: 横轴平移电机        ******/
 /******   1：竖轴平移电机        ******/
 /******   2：机械臂水平旋转电机  ******/
 /******   3：机械臂竖直俯仰电机  ******/
 /******   4：侧方抬升电机        ******/
 /******   5：侧方抬升电机        ******/
 /**************************************/

/*定义抬升电机pid变量，因为两个电机pid理论一致，定义变量可以方便调试*/
float p_tai=10000,d_tai = 1000;
float p_tai_angle=0.01,p_tai_angle_maxout=1.0,d_tai_angle=0;
/*定义横向移动pid变量*/
float p_heng = 10000,d_heng=200;
/*定义竖直移动pid变量*/
float p_shu = 10000,d_shu=1000,i_shu = 200;
/*定义机械臂水平旋转pid变量*/
float p_arm_horizon = 4000,d_arm_horizon = 0,max_out_arm_horizon = 13000,max_i_out_arm_horizon = 3000;
/*定义机械臂竖直俯仰pid变量*/
float p_arm_vertical = 7000,d_arm_vertical = 0,max_out_arm_vertical = 13000,max_i_out_arm_vertical = 3000;