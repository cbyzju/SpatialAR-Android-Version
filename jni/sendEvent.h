#ifndef _EVENT_
#define _EVENT_

  #include <android/log.h>
  #include <string.h>
  #include <errno.h>
  #include <stdio.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <fcntl.h>
  #include <linux/input.h>
  #include <sys/time.h>
  #include <sys/types.h>
  #include <unistd.h>

  #ifdef __cplusplus
  extern "C" {
  #endif
  namespace EVENT
  {
	      void single_touch(int fd, int pressure, int coord_x, int coord_y,int orientation,int angle);

		  void sys_report(int fd);

		  //单点触摸
		  int singletouch(int fd, int coord_x, int coord_y,int orientation,int angle);

		  //多点触摸
		  int multitouch(int fd, int coord_x1, int coord_y1, int coord_x2, int coord_y2,int orientation1,int angle1,int orientation2,int angle2);

          //in the air gesture
          int sweptright(int fd);
          int sweptleft(int fd);
          int chosen(int fd);

		  //释放触摸
		  int up(int fd);
		  int back(int fd);

		  //打开设备
		  int opendev();
  }

#ifdef __cplusplus
}
#endif

#endif