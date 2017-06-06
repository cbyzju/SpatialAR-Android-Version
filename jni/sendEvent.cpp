
  #include "sendEvent.h"

  #ifdef __cplusplus
  extern "C" {
  #endif

  #define ABS_MT_PRESSURE  0x3a
  #define ABS_MT_TOUCH_MAJOR 0x30
  #define ABS_MT_POSITION_X 0x35
  #define ABS_MT_POSITION_Y 0x36
  #define SYN_MT_REPORT 0x02

  #define ABS_MT_TOUCH_MINOR  0x31
  #define ABS_MT_ORIENTATION  0x34
  #define ABS_MT_WIDTH_MAJOR  0x32
  #define ABS_MT_WIDTH_MINOR  0x33

  #define LOG_TAG "event"
  #define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, fmt, ##args)
	  namespace EVENT{
		  void single_touch(int fd, int pressure, int coord_x, int coord_y,int orientation,int angle ) {
			  struct timeval tv;
			  gettimeofday(&tv, 0);

			  struct input_event event;
			  memset(&event, 0, sizeof(event));

			  event.type = EV_ABS;
			  event.code = ABS_MT_PRESSURE;
			  event.value = pressure;
			  event.time = tv;
			  write(fd, &event, sizeof(event));

			  event.type = EV_ABS;
			  event.code = ABS_MT_TOUCH_MAJOR;
			  event.value = 1;  //1
			  event.time = tv;
			  write(fd, &event, sizeof(event));


			  event.type = EV_ABS;
			  event.code = ABS_MT_POSITION_X;
			  event.value = coord_x;
			  event.time = tv;
			  write(fd, &event, sizeof(event));

			  event.type = EV_ABS;
			  event.code = ABS_MT_POSITION_Y;
			  event.value = coord_y;
			  event.time = tv;
			  write(fd, &event, sizeof(event));

              event.type = EV_ABS;
              event.code = ABS_MT_WIDTH_MAJOR ;
              event.value = orientation;
              event.time = tv;
              write(fd, &event, sizeof(event));

              event.type = EV_ABS;
              event.code = ABS_MT_WIDTH_MINOR;
              event.value = angle;
              event.time = tv;
              write(fd, &event, sizeof(event));

			  event.type = EV_SYN;
			  event.code = SYN_MT_REPORT;
			  event.value = 0;
			  event.time = tv;
			  write(fd, &event, sizeof(event));
		  }

		  void sys_report(int fd){
			  struct timeval tv;
			  gettimeofday(&tv, 0);

			  struct input_event event;
			  memset(&event, 0, sizeof(event));

			  event.type = EV_SYN;
			  event.code = SYN_REPORT;
			  event.value = 0;
			  event.time = tv;
			  write(fd, &event, sizeof(event));
		  }

		  //单点触摸
		  int singletouch(int fd, int coord_x, int coord_y,int orientation,int angle){

			  single_touch(fd, 1, coord_x, coord_y, orientation, angle);
			  sys_report(fd);

			  return 0;
		  }
		  //多点触摸
		  int multitouch(int fd, int coord_x1, int coord_y1, int coord_x2, int coord_y2,
		                 int orientation1,int angle1,int orientation2,int angle2){
			  single_touch(fd, 1, coord_x1, coord_y1, orientation1, angle1);
			  single_touch(fd, 2, coord_x2, coord_y2, orientation2, angle2);
			  sys_report(fd);

			  return 0;
		  }

		  //释放触摸
		  int up(int fd){
			  struct timeval tv;
			  gettimeofday(&tv, 0);
			  struct input_event event;
			  memset(&event, 0, sizeof(event));

			  event.type = EV_ABS;
			  event.code = ABS_MT_TOUCH_MAJOR;
			  event.value = 1;
			  event.time = tv;
			  write(fd, &event, sizeof(event));

			  event.type = EV_SYN;
			  event.code = SYN_MT_REPORT;
			  event.value = 0;
			  event.time = tv;
			  write(fd, &event, sizeof(event));

			  event.type = EV_SYN;
			  event.code = SYN_REPORT;
			  event.value = 0;
			  event.time = tv;
			  write(fd, &event, sizeof(event));

			  return 0;
		  }

          int sweptleft(int fd){

                int x=362;
                int y=300;
                up(fd);
                for(int i=0;i<2;i++){
                single_touch(fd, 1,x , y, 0, 400);
                sys_report(fd);
                x+=30;
               }
               up(fd);
               return 0;
          }
         int sweptright(int fd){
              int x=662;
              int y=300;
              up(fd);
              for(int i=0;i<2;i++){
                single_touch(fd, 1,x , y, 0, 400);
                sys_report(fd);
                x-=30;
              }
              up(fd);
              return 0;
         }
        //  int sweptright(int fd){
          //    up(fd);
          //    single_touch(fd, 1,512 , 300, 500, 0);
           //   sys_report(fd);
           //   up(fd);
           //   return 0;
         // }
         //  int sweptleft(int fd){
          //      up(fd);
            //    single_touch(fd, 1,512 , 300, 400, 0);
            //    sys_report(fd);
            //    up(fd);
             //   return 0;
           // }
         int back(int fd){
          single_touch(fd,1,640,670,0,400);
          sys_report(fd);
          up(fd);
          return 0;
         }
         int chosen(int fd){
              up(fd);
              single_touch(fd,1,640,360,0,400);
              sys_report(fd);
              up(fd);
              return 0;
         }


		  //打开设备
		  int opendev(){
			  int fd_touch = open("/dev/input/event1", O_RDWR);
			  if (fd_touch <= 0) {
				  LOGE("open touch error:%s", strerror(errno));
				  LOGE("open touch error:%d", fd_touch);
				  return -2;
			  }
			  LOGE("open touch error:%s", "2");
			  return fd_touch;
		  }
	  }

#ifdef __cplusplus
}
#endif
