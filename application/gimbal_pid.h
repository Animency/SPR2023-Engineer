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
float p_tai=2700					,d_tai = 11.3									,max_out_tai = 8000;  //10000,1000
float p_tai_angle=0.035		,p_tai_angle_maxout=6.0				,d_tai_angle=0.0035;
/*定义横向移动pid变量*/
float p_heng = 3000				,d_heng=130;
float p_heng_angle = 0.08	,d_heng_angle=0.012						,max_out_heng_angle=6.0;
/*定义竖直移动pid变量*/
float p_shu = 3000				,d_shu=135										,i_shu = 200;
float p_shu_angle=0.1			,d_shu_angle=0.012						,max_out_shu_angle=6.0;
/*定义机械臂水平旋转pid变量*/
float p_arm_horizon = 2300				,d_arm_horizon = 15												,max_out_arm_horizon = 20000						,max_i_out_arm_horizon = 3000;
float p_arm_horizon_angle = 0.2		,d_arm_horizon_angle = 0.00/*0.0015*/		,i_arm_horizon_angle = 0.0							,max_out_arm_horizon_angle = 20					,max_i_out_arm_horizon_angle = 10;
/*定义机械臂竖直俯仰pid变量*/
float p_arm_vertical = 3200				,d_arm_vertical = 14						,max_out_arm_vertical = 8000						,max_i_out_arm_vertical = 3000;
float p_arm_vertical_angle = 0.1	,d_arm_vertical_angle = 0.01		,max_out_arm_vertical_angle = 4.0				,max_i_out_arm_vertical_angle = 0.2;
/*定义存矿上升下降电机pid变量*/
float p_ore_vertical = 2000				,d_ore_vertical = 0						,max_out_ore_vertical = 8000						,max_i_ore_arm_vertical = 0;
float p_ore_vertical_angle = 0		,d_ore_vertical_angle = 0			,max_ore_arm_vertical_angle = 0.0				,max_i_ore_arm_vertical_angle = 0.0;
