#!/usr/bin/env python

import sys
import rospy
from agario_sele.srv import Notify
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
from subprocess import call


class AgarIOBot:
    def __init__(self):
	rospy.init_node('agario_sele')

	# rospy.wait_for_service('stream_screen_img_switch')
	# self.switch_screen_img = rospy.ServiceProxy('stream_screen_img_switch', Notify)

	rospy.wait_for_service('agario_info_switch')
	self.switch_info = rospy.ServiceProxy('agario_info_switch', Notify)

        self.openGame()
        self.getGameElements()

        self.configGame()
        self.initGame()
	# while True:
	    # self.initGame()
	    # self.testFinished()
	    # time.sleep(10)

    def openGame(self):
        self.driver = webdriver.Firefox()
        self.driver.find_element_by_tag_name("html").send_keys(Keys.F11);
        self.driver.get("http://agar.io/")

    def getGameElements(self):
	self.b_config  = self.driver.find_element_by_xpath("//body/div[5]/div[3]/div[2]/div[1]/div[4]/div[1]/button[1]")
	self.cb_skin = self.driver.find_element_by_xpath("//body/div[5]/div[3]/div[2]/div[1]/div[5]/div[1]/div[2]/label[1]/input")
	self.cb_names = self.driver.find_element_by_xpath("//body/div[5]/div[3]/div[2]/div[1]/div[5]/div[1]/div[2]/label[2]/input")
        # self.cb_colors = self.driver.find_element_by_xpath("//body/div[5]/div[3]/div[2]/div[1]/div[5]/div[1]/div[2]/label[3]/input")
	# self.cb_mass = self.driver.find_element_by_xpath("//body/div[5]/div[3]/div[2]/div[1]/div[5]/div[1]/div[2]/label[4]/input")
	self.b_play = self.driver.find_element_by_xpath("//body/div[5]/div[3]/div[2]/div[1]/div[4]/div[1]/button[3]")
	self.b_continue = self.driver.find_element_by_xpath("//body/div[5]/div[3]/div[3]/button[1]")

    def botSwitch(self, activate):
	# response = self.switch_screen_img(activate)
	response = self.switch_info(activate)

    def configGame(self):
        self.b_config.click()
        self.cb_skin.click()
        self.cb_names.click()
        # self.cb_colors.click()
        # self.cb_mass.click()

    def initGame(self):
        self.b_play.click()
        self.botSwitch(1)

    def testFinished(self):
        a = 0
        while not self.b_continue.is_displayed() and a < 100:
            a = a+1

        self.botSwitch(0)
        self.b_continue.click()

    def stop(self):
	self.driver.close()

if __name__ == "__main__":
    AgarIOBot()
