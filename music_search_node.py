#!/usr/bin/env python3

import rospy
import threading
from std_msgs.msg import String
from selenium import webdriver # type: ignore
from selenium.webdriver.common.by import By # type: ignore
from selenium.webdriver.chrome.service import Service # type: ignore
from selenium.webdriver.chrome.options import Options # type: ignore
from webdriver_manager.chrome import ChromeDriverManager # type: ignore # For auto-managing ChromeDriver
from selenium.webdriver.support.ui import WebDriverWait # type: ignore
from selenium.webdriver.support import expected_conditions as EC # type: ignore
import time

class MusicSearchNode:
    def __init__(self):
        rospy.init_node('music_search_node', anonymous=True)
        self.search_query_pub = rospy.Publisher('music_search_query', String, queue_size=10)
        rospy.loginfo("Music Search Node is ready. Waiting for song requests...")
        self.browser_active = False  # State to track browser activity
        self.driver = None  # Selenium WebDriver instance
        self.subscribe_to_topics()

    def subscribe_to_topics(self):
        """Subscribe to necessary topics and allow re-subscription when needed."""
        rospy.Subscriber('song_name', String, self.get_user_input)
        rospy.Subscriber('gesture_detected', String, self.handle_video_control)

    def get_user_input(self, msg):
        user_input = msg.data.strip()
        if user_input.lower() == 'exit':
            if self.browser_active and self.driver:
                self.close_browser()
        else:
            self.publish_search_query(user_input)

    def publish_search_query(self, query):
        rospy.loginfo(f"Publishing search query: {query}")
        self.search_query_pub.publish(query)
        self.search_music(query)

    def search_music(self, msg):
        query = msg.strip()
        if not query:
            rospy.logwarn("Received an empty search query. Ignoring.")
            return

        rospy.loginfo(f"Received search query: {query}")
        search_url = f"https://www.youtube.com/results?search_query={query.replace(' ', '+')}"
        rospy.loginfo(f"Generated search URL: {search_url}")

        # Start a new thread to open the browser
        threading.Thread(target=self.open_browser, args=(search_url,)).start()

    def open_browser(self, search_url):
        rospy.loginfo(f"Attempting to open browser with URL: {search_url}")
        try:
            chrome_options = Options()
            chrome_options.add_argument("--no-sandbox")
            chrome_options.add_argument("--disable-dev-shm-usage")
            chrome_options.add_argument("--start-maximized")
            # chrome_options.add_argument("--headless")  # Uncomment for headless mode
            chrome_options.add_argument("--disable-gpu")

            # Setup ChromeDriver via WebDriverManager
            self.driver = webdriver.Chrome(service=Service(ChromeDriverManager().install()), options=chrome_options)
            self.browser_active = True
            self.driver.get(search_url)
            rospy.loginfo("Opened the YouTube search page.")

            # Automatically click the first video link
            first_video = self.driver.find_element(By.CSS_SELECTOR, "a.yt-simple-endpoint.style-scope.ytd-video-renderer")
            first_video.click()
            rospy.loginfo("Playing the first video.")

            # Start threads for ad skipping and monitoring video playback
            threading.Thread(target=self.skip_ads_loop).start()
            threading.Thread(target=self.monitor_video_end).start()

        except Exception as e:
            rospy.logerr(f"Failed to open Chrome with Selenium: {e}")
            self.browser_active = False

    def monitor_video_end(self):
        """Monitor video playback and close the browser when the video ends."""
        rospy.loginfo("Started monitoring video playback.")
        while self.browser_active:
            try:
                video_element = self.driver.find_element(By.TAG_NAME, "video")
                is_ended = self.driver.execute_script("return arguments[0].ended;", video_element)
                if is_ended:
                    rospy.loginfo("The video has ended. Closing the browser...")
                    self.close_browser()
                    break
            except Exception as e:
                rospy.logwarn(f"Failed to monitor video playback: {e}")
            time.sleep(1)

    def close_browser(self):
        """Close the browser and reset the state."""
        if self.driver:
            try:
                self.driver.quit()
                rospy.loginfo("Browser closed.")
            except Exception as e:
                rospy.logwarn(f"Failed to close the browser: {e}")
            finally:
                self.driver = None
                self.browser_active = False  # Allow re-subscription

    def handle_video_control(self, msg):
        if not self.driver:
            rospy.logwarn("Browser is not active. Cannot control video.")
            return

        command = msg.data.strip().lower()
        rospy.loginfo(f"Received command: {command}")
        try:
            video_element = self.driver.find_element(By.TAG_NAME, "video")
            if command == "pause":
                self.driver.execute_script("arguments[0].pause();", video_element)
                rospy.loginfo("Paused the video.")
            elif command == "resume":
                self.driver.execute_script("arguments[0].play();", video_element)
                rospy.loginfo("Resumed the video.")
            elif command == "volume up":
                current_volume = self.driver.execute_script("return arguments[0].volume;", video_element)
                new_volume = min(current_volume + 0.1, 1.0)
                self.driver.execute_script("arguments[0].volume = arguments[1];", video_element, new_volume)
                rospy.loginfo(f"Volume increased to {new_volume:.1f}")
            elif command == "volume down":
                current_volume = self.driver.execute_script("return arguments[0].volume;", video_element)
                new_volume = max(current_volume - 0.1, 0.0)
                self.driver.execute_script("arguments[0].volume = arguments[1];", video_element, new_volume)
                rospy.loginfo(f"Volume decreased to {new_volume:.1f}")
            elif command == "mute":
                self.driver.execute_script("arguments[0].volume = 0.0;", video_element)
                rospy.loginfo("Muted the video.")
            elif command == "unmute":
                self.driver.execute_script("arguments[0].volume = 1.0",video_element)
                rospy.loginfo("Volume is at maximum")    
    
            else:
                rospy.loginfo(f"Unknown command: {command}")
        except Exception as e:
            rospy.logwarn(f"Failed to control the video: {e}")

    def skip_ads_loop(self):
        """Continuously check and skip ads."""
        rospy.loginfo("Started ad-skipping loop.")
        while self.browser_active:
            try:
                skip_button = WebDriverWait(self.driver, 5).until(
                    EC.presence_of_element_located((By.CLASS_NAME, "ytp-ad-skip-button"))
                )
                if skip_button.is_displayed():
                    skip_button.click()
                    rospy.loginfo("Skipped an ad.")
            except Exception as e:
                rospy.logdebug(f"No skippable ad detected: {e}")
            time.sleep(1)


if __name__ == '__main__':
    try:
        MusicSearchNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass








