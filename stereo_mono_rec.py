import cv2
import depthai as dai
import numpy as np
import threading

from stereo_interface import StereoInterface
import time
import queue

def main():
    before_left_time_stamp = time.time()
    before_right_time_stamp = time.time()

    stereo_if = StereoInterface()

    HEIGHT = 720
    WIDTH = 720 // 9 * 16
    FPS = 30
    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    writer = cv2.VideoWriter("stereo_mono.avi", fourcc, FPS, (WIDTH*2, HEIGHT))

    left_que = queue.Queue()
    right_que = queue.Queue()

    end_signal = False

    def thread_write_frame():
        while(True):
            if not left_que.empty() and not right_que.empty():
                left_que_frame = left_que.get()
                right_que_frame = right_que.get()
                con = np.concatenate([left_que_frame, right_que_frame], 1)
                con = cv2.cvtColor(con, cv2.COLOR_GRAY2BGR)
                writer.write(con)
            
            if end_signal is True:
                break

    write_frame_thread = threading.Thread(target=thread_write_frame)
    write_frame_thread.start()

    def thread_get_frame():
        while(True):
            left = stereo_if.get_left()
            right = stereo_if.get_right_frame()
    
            if left is not False:
                cv2.imshow("left", left)
                left_time = time.time()
                print(f"left frame per second : {1 / (left_time - before_left_time_stamp)}")
                before_left_time_stamp = left_time
                left_que.put(left)
    
            if right is not False:
                cv2.imshow("right", right)
                right_time = time.time()
                print(f"right frame per second : {1 / (right_time - before_right_time_stamp)}")
                before_right_time_stamp = right_time
                right_que.put(right)
    
    
            if not left_que.empty() and not right_que.empty():
                left_que_frame = left_que.get()
                right_que_frame = right_que.get()
                con = np.concatenate([left_que_frame, right_que_frame], 1)
                con = cv2.cvtColor(con, cv2.COLOR_GRAY2BGR)
                writer.write(con)
    
            if cv2.waitKey(1) == ord('q'):
                end_signal = True
                thread_write_frame.join()
                break
            
        writer.release()

    
        

if __name__ == '__main__':
    main()