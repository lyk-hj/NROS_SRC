//#include "Thread.hpp"
//#include <cstdio>
//#include <opencv2/opencv.hpp>
//#include <chrono>
//
//
//using namespace cv;
//
//bool is_continue = true;
//
//typedef struct form
//{
//    vector<Armor> armors;
//    float data[4];
//    int mode;
//    bool dat_is_get;
//    double tim;
//}form;
//
////one2two
//bool data_get;
//int mode/* = 0x22*/;
//double tim;
//Mat src;
//robot_state robot;
////two2three
//form send_data;
//
//Mat ka_src;
////Mat ka_src_get;
//
//SerialPort port("/dev/ttyUSB");
//
//void* Build_Src(void* PARAM)
//{
//    bool lin_is_get/* = true*/;
//    int mode_temp;
//    int color;
//    Mat get_src;
//    float lin[4];
//    double time_temp;
//
//    auto camera_warper = new Camera;
//    port.initSerialPort();
//
//	printf("imu_camera_open\n");
//
//    if (camera_warper->init())
//	{
//		printf("1-real\n");
//		while (is_continue && !(waitKey(10) == 27))
//		{
//
//			if (camera_warper->read_frame_rgb())
//			{
////				printf("1\n");
//
//				get_src = camera_warper->src.clone();
//                lin_is_get = port.get_Mode1(mode_temp, lin[0], lin[1], lin[2], lin[3],color);
//				time_temp = (double)getTickCount();
//
//				pthread_mutex_lock(&mutex_new);
//				{
//                    get_src.copyTo(src);
//                    robot.updateData(lin,color);
//                    mode = mode_temp;
//                    tim = time_temp;
//                    is_start = true;
//                    data_get = lin_is_get;
//                    pthread_cond_signal(&cond_new);
//                    pthread_mutex_unlock(&mutex_new);
//                    imshow("src",src);
//                    camera_warper->release_data();
//				}
//				//camera_warper->record_start();
//				//camera_warper->camera_record();
//			}
//			else
//			{
//				src = cv::Mat();
//			}
//
//		}
//		camera_warper->~Camera();
//		pthread_mutex_unlock(&mutex_new);
//		is_continue = false;
//	}
//	else
//	{
//		printf("No camera!!\n");
//		is_continue = false;
//	}
//}
//
//void* Armor_Kal(void* PARAM)
//{
//    ArmorDetector Detect;
//    std::vector<Armor> Targets;
//
//	Mat src_copy;
//    double time_temp;
//	int mode_temp;
//	bool color_get;
//
//	sleep(2);
//	printf("Armor_open\n");
//	while (is_continue)
//	{
//		pthread_mutex_lock(&mutex_new);
//
//		while (!is_start) {
//			pthread_cond_wait(&cond_new, &mutex_new);
//		}
//
//		is_start = false;
//
//		src.copyTo(src_copy);
//        mode_temp = mode;
//        color_get = data_get;
//        time_temp = tim;
//		//imshow("src_copy",src_copy);
//
//		pthread_mutex_unlock(&mutex_new);
//        if(color_get)
//        {
//            if (send_data.mode == 0x21)
//            {
//                Detect.updateData(send_data.data,color_get);
//                Targets = Detect.autoAim(src_copy);
//                pthread_mutex_lock(&mutex_ka);
//                send_data.armors = Targets;
//                send_data.data[0] = Detect.ab_pitch;
//                send_data.data[1] = Detect.ab_yaw;
//                send_data.data[2] = Detect.ab_roll;
//                send_data.data[3] = Detect.bullet_speed;
//                send_data.mode = mode_temp;
//                send_data.dat_is_get = color_get;
//                send_data.tim = time_temp;
//                is_ka = true;
//                src_copy.copyTo(ka_src);
//                pthread_cond_signal(&cond_ka);
//                pthread_mutex_unlock(&mutex_ka);
//            }
//        }
//
//	}
//}
// // todo: 给这个线程加入图像src_copy，时间传递
//void* Kal_predict(void* PARAM)
//{
//    Mat src_copy;
//    VisionData vdata;
//	vector<Armor> armors;
//    ArmorTracker Track;
//	int mode_temp;
//	int angle_get;
//    double time_temp;
//
//	sleep(3);
//	printf("kal_open\n");
//    float send_pitch,send_yaw;
//    int pan_wu = 0;
//
//	while (is_continue)
//	{
//		pthread_mutex_lock(&mutex_ka);
//		while (!is_ka)
//        {
//			pthread_cond_wait(&cond_ka, &mutex_ka);
//		}
//
//		is_ka = false;
//        src.copyTo(src_copy);
//
//		angle_get = send_data.dat_is_get;
//		mode_temp = send_data.mode;
//        armors = send_data.armors;
//        time_temp = send_data.tim;
//
//		pthread_mutex_unlock(&mutex_ka);
//
//		if(angle_get)
//		{
//			if (mode_temp == 0x21)
//			{
//                Track.AS.updateData(send_data.data);
////                Track.AS.init(send_data.data[2],send_data.data[0],send_data.data[1],send_data.data[3]);
//
//				if (Track.locateEnemy(src_copy,armors,time_temp))
//				{
//                    send_pitch = Track.pitch;
//                    send_yaw = Track.yaw;
//					vdata = { -send_pitch, -send_yaw, 0x31 };
//					printf("yaw:%f\npitch:%f\n", -Track.yaw, -Track.pitch);
//					port.TransformData(vdata);
//					port.send();
//					pan_wu = 0;
//				}
//				else
//				{
//					if(pan_wu<=12)
//					{
//
//						vdata = { -send_pitch, -send_yaw, 0x31 };
//						printf("yaw:%f\npitch:%f\npan_wu:%d\n",-send_yaw, -send_pitch,pan_wu);
//						port.TransformData(vdata);
//						port.send();
//						pan_wu++;
//					}
//                    else
//					{
//                        send_yaw = 0.0 - send_data.data[1];
//                        send_pitch = 0.0 - send_data.data[0];
////						ka.sp_reset(kf);
//						vdata = { -send_pitch, -send_yaw, 0x32 };
//						//printf("real none!!");
//						//printf("chong\n");
//						port.TransformData(vdata);
//						port.send();
//					}
//				}
//			}
//		}
//	}
//}
//
//
