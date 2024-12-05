#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32, Int32MultiArray
from threading import Timer
import pygame  # For MP3 playback

class PomodoroTimer:
    def __init__(self):
        rospy.init_node('pomodoro_timer', anonymous=True)
        
        self.state_pub = rospy.Publisher('/pomodoro/state', String, queue_size=10)
        self.time_pub = rospy.Publisher('/pomodoro/time', Int32, queue_size=10)
        self.notification_pub = rospy.Publisher('/pomodoro/notification', String, queue_size=10)
        self.log_pub = rospy.Publisher('/pomodoro/log', String, queue_size=10)
        self.loops_remaining_pub = rospy.Publisher('/pomodoro/loops_remaining', Int32, queue_size=10)

        self.control_sub = rospy.Subscriber('/pomodoro/control', String, self.control_callback)
        self.custom_time_sub = rospy.Subscriber('/pomodoro/custom_time', Int32MultiArray, self.update_custom_times)
        self.loop_sub = rospy.Subscriber('/pomodoro/loops', Int32, self.update_loops)

        self.state = "stopped"  # Possible states: stopped, running, paused
        self.work_time = 25 * 60  # 25 minutes default
        self.break_time = 5 * 60  # 5 minutes default
        self.remaining_time = self.work_time
        self.current_cycle = "work"  # "work" or "break"
        self.total_loops = 1  # Default is 1 loop (work + break cycle)
        self.completed_loops = 0
        self.timer = None

        # Initialize Pygame for MP3 playback
        pygame.init()
        self.mp3_file_break = "/home/tey/catkin_ws/src/edu_robot/audio/timer.mp3"  # Replace with actual path for break completion
        self.mp3_file_session = "/home/tey/catkin_ws/src/edu_robot/audio/timer.mp3"  # Replace with actual path for session completion

        self.log_message("Pomodoro Timer Node with MP3 Feature Started")

    def log_message(self, message):
        timestamped_message = f"[INFO] [{rospy.get_time()}]: {message}"
        rospy.loginfo(message)
        self.log_pub.publish(timestamped_message)

    def control_callback(self, msg):
        command = msg.data.lower()
        if command == "start":
            self.start_timer()
        elif command == "pause":
            self.pause_timer()
        elif command == "reset":
            self.reset_timer()

    def update_custom_times(self, msg):
        self.work_time = msg.data[0] * 60
        self.break_time = msg.data[1] * 60
        self.remaining_time = self.work_time
        self.log_message(f"Updated times: Work={self.work_time // 60} mins, Break={self.break_time // 60} mins")

    def update_loops(self, msg):
        self.total_loops = msg.data
        self.completed_loops = 0
        self.loops_remaining_pub.publish(self.total_loops)  # Publish total loops as remaining at start
        self.log_message(f"Updated loop count: {self.total_loops}")

    def start_timer(self):
        if self.state == "running":
            return
        self.state = "running"
        self.timer_tick()
        self.log_message("Timer started.")
        self.publish_state()

    def pause_timer(self):
        if self.state != "running":
            return
        self.state = "paused"
        if self.timer:
            self.timer.cancel()
        self.log_message("Timer paused.")
        self.publish_state()

    def reset_timer(self):
        self.state = "stopped"
        self.remaining_time = self.work_time
        self.current_cycle = "work"
        self.completed_loops = 0
        self.loops_remaining_pub.publish(self.total_loops)  # Reset remaining loops
        if self.timer:
            self.timer.cancel()
        self.log_message("Timer reset.")
        self.publish_state()
        self.publish_time()

    def timer_tick(self):
        if self.state != "running":
            return

        if self.remaining_time > 0:
            self.remaining_time -= 1
            self.publish_time()
            self.timer = Timer(1, self.timer_tick)
            self.timer.start()
        else:
            self.complete_cycle()

    def complete_cycle(self):
        if self.current_cycle == "work":
            self.current_cycle = "break"
            self.remaining_time = self.break_time
            self.log_message("Work session completed. Starting break.")
            self.play_mp3(self.mp3_file_break)  # Play MP3 for break completion
        else:
            self.completed_loops += 1
            remaining_loops = self.total_loops - self.completed_loops
            self.loops_remaining_pub.publish(remaining_loops)  # Publish remaining loops
            if self.completed_loops < self.total_loops:
                self.current_cycle = "work"
                self.remaining_time = self.work_time
                self.log_message(f"Break session completed. Starting next loop ({self.completed_loops}/{self.total_loops}).")
                self.play_mp3(self.mp3_file_break)  # Play MP3 for break completion
            else:
                self.notify_completion()
                self.reset_timer()
                return

        self.publish_time()
        self.timer_tick()

    def notify_completion(self):
        notification_message = "All loops completed! Timer finished."
        self.log_message(notification_message)
        self.notification_pub.publish(notification_message)
        self.play_mp3(self.mp3_file_session)  # Play MP3 for session completion

    def play_mp3(self, file_path):
        try:
            pygame.mixer.init()
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            self.log_message(f"Playing MP3 notification: {file_path}")
        except Exception as e:
            self.log_message(f"Error playing MP3: {e}")

    def publish_state(self):
        self.state_pub.publish(self.state)

    def publish_time(self):
        self.time_pub.publish(self.remaining_time)

if __name__ == "__main__":
    try:
        PomodoroTimer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass







  