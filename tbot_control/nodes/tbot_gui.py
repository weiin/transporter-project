#!/usr/bin/env python
import roslib; roslib.load_manifest('tbot_control')
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Bool
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import GetMap
import actionlib
from move_base_msgs.msg import MoveBaseActionResult

import matplotlib.pyplot as plt
import numpy
from PIL import Image
from os import path

import random
import wx
import time
from threading import *



# Define notification event for thread updation
EVT_RESULT_ID = wx.NewId()
EVT_REPAINT = wx.NewId()

# Define Waypoint + Path list
WP_cnt = 0
WAYPOINTS = []
PATH_cnt = 0
PATH = []

# Define Goal
GOAL_SET = 0
GOAL_CURRENT = 0
GOAL_REACHED = 0
GOAL_RUN = 0

# Define GUI states
class STATE:
	NIL=0
	ADD_WP=1
	DEL_WP=2
	ADD_PATH=3
	RESET_LOC=4
	
STATE_flag = STATE.NIL

# Define global Robot Position
ESTOP = False
ROBOT_CONNECTED = 0
ROBOT_POS = [0,0]
MAP_RES = 0.05
MAP_SIZE = [768,864]
MAP_ORIGIN = [-25.0,-23.39999999999]

IMG_SCALE = 0
IMG_ORIGIN = [0,0]


def EVT_RESULT(win, func):
    """Define Result Event."""
    win.Connect(-1, -1, EVT_RESULT_ID, func)

class ResultEvent(wx.PyEvent):
    """Simple event to carry arbitrary result data."""
    def __init__(self, data):
        """Init Result Event."""
        wx.PyEvent.__init__(self)
        self.SetEventType(EVT_RESULT_ID)
        self.data = data

def goalCallBack(msg) :
	#print 'Goal Result Callback'
	#print msg
	global GOAL_REACHED
	if GOAL_SET == 1:
		if msg.status.status == 3:
			GOAL_REACHED = 3
			
		elif msg.status.status == 4:
			GOAL_REACHED = 4
			#print 'Goal Aborted'
		
def setGoal(goal) :
	print 'Goal Setting...'
	pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, latch=True)

	goalpose = PoseStamped()
	goalpose.header.stamp = rospy.Time.now()
	goalpose.header.frame_id = '/map'

	goalpose.pose.position.x = goal[0]
	goalpose.pose.position.y = goal[1]
	goalpose.pose.position.z = 0
	goalpose.pose.orientation.x = 0
	goalpose.pose.orientation.y = 0
	goalpose.pose.orientation.z = 0
	goalpose.pose.orientation.w = 1

# weiin: changed publisher to latched, so no need for multiple publish
#	counter = 0;
#	while not rospy.is_shutdown():
		
	pub.publish(PoseStamped(goalpose.header,goalpose.pose))
#		rospy.sleep(0.1)
#		counter += 1
#		if counter > 5 :
#			break
	print 'Goal Send'
def initialPose() :
	pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,latch=True)

	initialpose = PoseWithCovarianceStamped()
	initialpose.header.frame_id = '/map'
	initialpose.pose.pose.position.x = 0
	initialpose.pose.pose.position.y = 0
	initialpose.pose.pose.position.z = 0
	initialpose.pose.pose.orientation.x = 0
	initialpose.pose.pose.orientation.y = 0
	initialpose.pose.pose.orientation.z = 0
	initialpose.pose.pose.orientation.w = -1
	initialpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25,\
						 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
						 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0,\
						 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#	counter = 0
#	while not rospy.is_shutdown():
		
	pub.publish(PoseWithCovarianceStamped(initialpose.header,initialpose.pose))
#		rospy.sleep(0.1)
#		counter += 1
#		if counter > 3 :
#			break
		
def eStop(status) :
	pub = rospy.Publisher('/cmd_estop', Bool)

	
	counter = 0
	while not rospy.is_shutdown():
		
		pub.publish(status)
		rospy.sleep(0.1)
		counter += 1
		if counter > 3 :
			break
		
		
# Thread class that updates robot position
class AcquirerThread(Thread):
    	"""Worker Thread Class."""
    	def __init__(self, notify_window):
		Thread.__init__(self)
		self._notify_window = notify_window
		self._want_abort = 0
		# This starts the thread running on creation, but you could
		# also make the GUI thread responsible for calling this
		self.daemon = True
		self.start()
		print 'TF thread init'

	def run(self):
		
		while ROBOT_CONNECTED == 0:
			rospy.sleep(0.5)
			
		"""Run Worker Thread."""
		tf_listener = tf.TransformListener()
	
		print 'Starting TF Listener....'
		
		tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(10.0))
		
    		while not rospy.is_shutdown():
        		try:
            			
            			tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
				(trans,rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
				#print (trans,rot)
				wx.PostEvent(self._notify_window, ResultEvent(trans))
				rospy.sleep(0.2)

			except (tf.LookupException, tf.ConnectivityException):
				#print 'tf connectivity exception'
				rospy.sleep(0.2)
				continue
			if self._want_abort == 1:
				print 'Transform Listener stopped'
				return
			

	def abort(self):
		"""abort worker thread."""
		# Method for use by main thread to signal an abort
		self._want_abort = 1
		
# Thread class that sends goal position
class GoalThread(Thread):
    	"""Worker Thread Class."""
    	def __init__(self, notify_window):
		Thread.__init__(self)
		self._notify_window = notify_window
		self._want_abort = 0
		# This starts the thread running on creation, but you could
		# also make the GUI thread responsible for calling this
		self.daemon = True
		self.start()
		print 'Goal thread init'

	def run(self):
		
		while ROBOT_CONNECTED == 0:
			rospy.sleep(0.5)
			
		
	
		print 'Goal transmission started...'
		
		#tf_listener.waitForTransform('/map', '/base_link', 0, rospy.Duration(4.0))
		
		
		while not rospy.is_shutdown():
			rospy.sleep(0.2)
			sub = rospy.Subscriber("move_base/result", MoveBaseActionResult, goalCallBack)
			global PATH_cnt, PATH
			global GOAL_REACHED, GOAL_SET
			while PATH_cnt > 0 and GOAL_SET > 0:
				#print 'Goal transmission started...'
				GOAL_REACHED = 0
				goal = [0,0]
				goal[0] = WAYPOINTS[PATH[0]][0]*MAP_RES + MAP_ORIGIN[0]
				goal[1] = (MAP_SIZE[1]*MAP_RES  + MAP_ORIGIN[1]) - WAYPOINTS[PATH[0]][1]*MAP_RES
				print goal
				setGoal(goal)
				rospy.sleep(0.5)
				while GOAL_REACHED != 3:
						
					if GOAL_REACHED == 4:
						print "Cant find a path to the goal. Abort"
						break
					rospy.sleep(0.2)
				
				
				
				GOAL_REACHED = 0	
				
				if PATH_cnt > 0:
					PATH.pop(0)
					PATH_cnt -= 1
			
			if GOAL_SET > 0:
				GOAL_SET = 0
				result = ['complete',0] #complete
				wx.PostEvent(self._notify_window, ResultEvent(result))
			
					
			if self._want_abort == 1:
				print 'Goal transmission stopped'
				return

			

	def abort(self):
		"""abort worker thread."""
		# Method for use by main thread to signal an abort
		self._want_abort = 1


# ------------------------------------------------------------------------------
# --------------------------    MAP DISPAY    ----ros you requested a transform but t------------------------------
# ------------------------------------------------------------------------------

class MyCanvas(wx.ScrolledWindow):
	
	
	def __init__(self, parent, id = -1, size = wx.DefaultSize):

		wx.ScrolledWindow.__init__(self, parent, id, (0, 0), size=size, style=wx.SUNKEN_BORDER)
		self.filename = icons_path + 'mapimage.png'
		self.imageArray = []
		self.image = wx.Image(self.filename)
		
		for i in range(10):
			self.imageArray.append(wx.BitmapFromImage(self.image.Scale(self.image.GetWidth() * (1+i*0.4), self.image.GetHeight()*(1+i*0.4))))
			print [self.imageArray[i].GetWidth() ,self.imageArray[i].GetHeight()]
		self.robotBmp =  wx.BitmapFromImage(wx.Image(icons_path + 'robot.png'))
		self.wpBmp =  wx.BitmapFromImage(wx.Image(icons_path + 'wp.png'))
		self.wpnewBmp =  wx.BitmapFromImage(wx.Image(icons_path + 'wpnew.png'))
		self.wprobotBmp =  wx.BitmapFromImage(wx.Image(icons_path + 'wprobot.png'))
		self.w = self.image.GetWidth()
		self.h = self.image.GetHeight()
		self.imageOrigin = [0,0]
		self.robotPos = [self.w/2,self.h/2]
		self.winSize = [1000,800]
		
		global IMG_ORIGIN
		IMG_ORIGIN = self.imageOrigin
		
		global ROBOT_POS 
		ROBOT_POS = self.robotPos
		#WAYPOINTS.append(ROBOT_POS)
		
		self.SetVirtualSize((self.w, self.h))
		self.SetScrollRate(20,20)
		self.SetBackgroundColour(wx.Colour(0,0,0))

		if BUFFERED:
		    # Initialize the buffer bitmap.  No real DC is needed at this point.
		    self.buffer = wx.EmptyBitmap(self.winSize[0], self.winSize[1])
		    dc = wx.BufferedDC(None, self.buffer)
		    dc.Clear()
		    
		    self.DoDrawing(dc, self.imageOrigin)

		self.Bind(wx.EVT_PAINT, self.OnPaint)
		self.Bind(wx.EVT_SIZE, self.OnSize)
		self.Bind(wx.EVT_MOUSEWHEEL, self.OnScroll)
		
		self.Centre()
		# bind left click event to OnClick method
		self.Bind(wx.EVT_LEFT_UP, self.OnClick)
		
	def UpdateImages(self, filename):
		self.image = wx.Image(filename)
		self.imageArray = []
		for i in range(10):
			self.imageArray.append(wx.BitmapFromImage(self.image.Scale(self.image.GetWidth() * (1+i*0.4), self.image.GetHeight()*(1+i*0.4))))
			print [self.imageArray[i].GetWidth() ,self.imageArray[i].GetHeight()]
		dc = wx.BufferedDC(None, self.buffer)
		self.DoDrawing(dc,IMG_ORIGIN)
		self.Refresh()
		
		
	def OnPaint(self, event):
		
		if BUFFERED:
		    
		    dc = wx.BufferedPaintDC(self, self.buffer, wx.BUFFER_VIRTUAL_AREA)

	def DoDrawing(self, dc, origin):
		
		global ROBOT_POS
		self.winSize = [self.imageArray[int(IMG_SCALE)].GetWidth(),self.imageArray[int(IMG_SCALE)].GetHeight()]
		dc.Clear()
		dc.DrawBitmap(self.imageArray[int(IMG_SCALE)], origin[0], origin[1])
		dc.SetPen(wx.Pen('orange', 2))		
		self.DrawLn(dc, ROBOT_POS[0], ROBOT_POS[1])
		self.DrawPt(dc, ROBOT_POS[0], ROBOT_POS[1])

	def DrawPt(self, dc, x, y, size = 40):
		factor = float(self.imageArray[int(IMG_SCALE)].GetWidth())/float(self.imageArray[0].GetWidth())
		#print [x,y,factor,IMG_SCALE, IMG_ORIGIN]
		x = (x*factor) + IMG_ORIGIN[0] 
		y = (y*factor) + IMG_ORIGIN[1]
		
		dc.DrawBitmap(self.robotBmp, x-23, y-45)
		
		
	def DrawLn(self, dc, x, y, size = 40):
		global PATH_cnt
		factor = float(self.imageArray[int(IMG_SCALE)].GetWidth())/float(self.imageArray[0].GetWidth())
		cnt = 1
		if PATH_cnt > 0:
			point1 = [(ROBOT_POS[0]*factor) + IMG_ORIGIN[0] ,(ROBOT_POS[1]*factor) + IMG_ORIGIN[1]]
			point2 = [(WAYPOINTS[PATH[0]][0]*factor) + IMG_ORIGIN[0] ,(WAYPOINTS[PATH[0]][1]*factor) + IMG_ORIGIN[1]]
			
			dc.DrawLine(point1[0], point1[1], point2[0], point2[1])
		while cnt < PATH_cnt :
			point1 = [(WAYPOINTS[PATH[cnt-1]][0]*factor) + IMG_ORIGIN[0] ,(WAYPOINTS[PATH[cnt-1]][1]*factor) + IMG_ORIGIN[1]]
			point2 = [(WAYPOINTS[PATH[cnt]][0]*factor) + IMG_ORIGIN[0] ,(WAYPOINTS[PATH[cnt]][1]*factor) + IMG_ORIGIN[1]]
			dc.DrawLine(point1[0], point1[1], point2[0], point2[1])
			cnt += 1;
		cnt = 0
		while cnt < WP_cnt :
			point1 = [(WAYPOINTS[cnt][0]*factor) + IMG_ORIGIN[0] ,(WAYPOINTS[cnt][1]*factor) + IMG_ORIGIN[1]]
			#point2 = WAYPOINTS[cnt+1]self.wpnewBmp =  wx.BitmapFromImage(wx.Image(icons_path + 'wpnew.png'))
			path_flag = 0
			for x in PATH:
				if x == cnt:
					path_flag = 1
			if path_flag == 1:
				dc.DrawBitmap(self.wpnewBmp, point1[0]-10, point1[1]-10)
			else:
				dc.DrawBitmap(self.wpBmp, point1[0]-11, point1[1]-11)
			#dc.DrawBitmap(self.wpBmp, point2[0]-11, point2[1]-11)
			cnt += 1;
		
			
	def OnClick(self, event):
		pos = self.CalcUnscrolledPosition(event.GetPosition())
		factor = float(self.imageArray[int(IMG_SCALE)].GetWidth())/float(self.imageArray[0].GetWidth())
		if pos[0]-20 < IMG_ORIGIN[0] or pos[0]+20 > IMG_ORIGIN[0] + self.imageArray[int(IMG_SCALE)].GetWidth() or pos[1]-20 < IMG_ORIGIN[1] or pos[1]+20 > IMG_ORIGIN[1] + self.imageArray[int(IMG_SCALE)].GetHeight() :
			return
		
		pos = [(pos[0]-IMG_ORIGIN[0])/factor ,(pos[1] - IMG_ORIGIN[1])/factor]
		
		
		global WP_cnt
		global PATH_cnt
		global PATH
		if STATE_flag == STATE.RESET_LOC:
			global ROBOT_POS
			#ROBOT_POS = pos
			#pos_robotframe = [0,0]
			#initialPose(pos_robotframe)
			
		elif STATE_flag == STATE.ADD_WP:
			WP_cnt += 1
			WAYPOINTS.append(pos)
		elif STATE_flag == STATE.ADD_PATH:
			
			cnt = 0
			while cnt < WP_cnt :
				if pos[0]-20 < WAYPOINTS[cnt][0] and pos[0]+20 > WAYPOINTS[cnt][0] and pos[1]-20 < WAYPOINTS[cnt][1] and pos[1]+20 > WAYPOINTS[cnt][1] :
					duplicate_flag = 0
					for x in PATH:
						if x == cnt:
							duplicate_flag = 1
					if duplicate_flag == 1:
						break
					PATH_cnt += 1
					PATH.append(cnt)
				cnt += 1
		elif STATE_flag == STATE.DEL_WP:
			
			cnt = 0
			while cnt < WP_cnt :
				if pos[0]-20 < WAYPOINTS[cnt][0] and pos[0]+20 > WAYPOINTS[cnt][0] and pos[1]-20 < WAYPOINTS[cnt][1] and pos[1]+20 > WAYPOINTS[cnt][1] :
					
					inpath_flag = 0
					inpath_cnt = 0
					inpath_idx = 0
					for x in PATH:
						if x == cnt:
							inpath_flag = 1
							inpath_idx = inpath_cnt
						elif x > cnt:
							PATH[inpath_cnt] -= 1
						inpath_cnt += 1
					if inpath_flag == 1:
						PATH_cnt -= 1
						PATH.pop(inpath_idx)		
					WP_cnt -= 1
					WAYPOINTS.pop(cnt)
					
				cnt += 1
		#print 'clicked at %d, %d' %(pos.x, pos.y)
		#print WP_cnt 
		#print STATE_flag
		dc = wx.BufferedDC(None, self.buffer)
		self.DoDrawing(dc,IMG_ORIGIN)
		self.Refresh()
		
	def OnSize(self, event):
		a = 1
		#print (str(event.GetSize()))
		
	def OnScroll(self, event):
		global IMG_ORIGIN , IMG_SCALE
		prev_Scale = IMG_SCALE
		prev_Origin = IMG_ORIGIN
		origin = [0,0]
		new_mouseFocus = [0,0]
		dc = wx.BufferedDC(None, self.buffer)
		
		mouseFocus = self.CalcUnscrolledPosition(event.GetPosition())
		wheelRot = event.GetWheelRotation()
		self.Refresh()#print 'Scroll'
		if wheelRot > 0 and IMG_SCALE < 9:
			IMG_SCALE += 1
			new_mouseFocus[0] = ((mouseFocus[0] - IMG_ORIGIN[0]) * self.imageArray[int(IMG_SCALE)].GetWidth())/self.imageArray[int(prev_Scale)].GetWidth()
			new_mouseFocus[1] = ((mouseFocus[1] - IMG_ORIGIN[1]) * self.imageArray[int(IMG_SCALE)].GetHeight())/self.imageArray[int(prev_Scale)].GetHeight()
			origin[0] = IMG_ORIGIN[0]  - (new_mouseFocus[0] - (mouseFocus[0] - IMG_ORIGIN[0]))
			origin[1] = IMG_ORIGIN[1]  - (new_mouseFocus[1] - (mouseFocus[1] - IMG_ORIGIN[1]))
			IMG_ORIGIN = origin
		elif wheelRot < 0 and IMG_SCALE > 0:
			IMG_SCALE -= 1
			new_mouseFocus[0] = ((mouseFocus[0] - IMG_ORIGIN[0]) * self.imageArray[int(IMG_SCALE)].GetWidth())/self.imageArray[int(prev_Scale)].GetWidth()
			new_mouseFocus[1] = ((mouseFocus[1] - IMG_ORIGIN[1]) * self.imageArray[int(IMG_SCALE)].GetHeight())/self.imageArray[int(prev_Scale)].GetHeight()
			origin[0] = IMG_ORIGIN[0]  - (new_mouseFocus[0] - (mouseFocus[0] - IMG_ORIGIN[0]))
			origin[1] = IMG_ORIGIN[1]  - (new_mouseFocus[1] - (mouseFocus[1] - IMG_ORIGIN[1]))
			IMG_ORIGIN = origin
		#print [IMG_ORIGIN, mouseFocus, prev_Scale, IMG_SCALE]
		
		
		dc = wx.BufferedDC(None, self.buffer)
		self.DoDrawing(dc,IMG_ORIGIN)
		self.Refresh()
	
#-------------------------------------------------------------------------------
#--------------------    WINDOW    ---------------------------------------------
#-------------------------------------------------------------------------------

class navWindow(wx.Frame):
	def __init__(self, parent, id, title):
        	wx.Frame.__init__(self, parent, id, title, size=(1000, 600))
		#self.SetSizeHints(800,400, -1,-1)
		
		self.iconFile = icons_path + 'robot.png'
		self.icon1 = wx.Icon(self.iconFile, wx.BITMAP_TYPE_PNG)
		self.SetIcon(self.icon1)
		
		self.acquirer_thread = None
			
		self.statusBar = self.CreateStatusBar()
		#-----------------------------------------------------------------------
		
		self.topSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.tabSizer = wx.BoxSizer(wx.VERTICAL)
		
		#----   Image Box Code  ------------------------------------------------
		self.canvas = MyCanvas(self, -1)
		
        	self.canvas.SetBackgroundColour(wx.Colour(200, 200, 200))
		self.topSizer.Add(self.canvas,3,wx.EXPAND|wx.ALL,10)
		self.topSizer.Add(self.tabSizer,1,wx.EXPAND,10)
		
		#-----------------    Static Box for Path Planning
		self.staticBoxPath = wx.StaticBox(self, -2, 'Planner', size=(100, 100))
		self.font = wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD)
		self.heading1 = wx.StaticText(self, -1, 'Route plan', (130, 15))
		self.heading1.SetFont(self.font)
		self.staticBoxPath.SetFont(self.font)
		self.sLine1 = wx.StaticLine(self, -1, (25, 50), (100,1))
		self.sLine2 = wx.StaticLine(self, -1, (25, 50), (100,1))
		self.heading2 = wx.StaticText(self, -1, 'Way-point ', (130, 15))
		self.heading2.SetFont(self.font)
		
		self.btnNewPath = wx.Button(self, -1, 'New Path', (50, 50), (60, -1))
		self.btnClearPath = wx.Button(self, -1, 'Clear', (50, 50), (60, -1))
		self.btnUndoPath = wx.Button(self, -1, 'Undo', (50, 50), (60, -1))
		self.btnAddWaypoint = wx.Button(self, -1, 'Add Point', (50, 50), (60, -1))
		self.btnDelWaypoint = wx.Button(self, -1, 'Delete Point', (50, 50), (60, -1))
		self.sbSizer1 = wx.StaticBoxSizer(self.staticBoxPath, wx.VERTICAL)
		self.sbSizer1.AddSpacer((10, 4))
		self.sbSizer1.Add(self.heading1,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer1.Add(self.sLine1,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer1.Add(self.btnNewPath,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.pathSizer = wx.BoxSizer(wx.HORIZONTAL)
		self.pathSizer.Add(self.btnClearPath,0,wx.EXPAND|wx.RIGHT,5)
		self.pathSizer.Add(self.btnUndoPath,0,wx.EXPAND,0)
		self.sbSizer1.Add(self.pathSizer,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer1.AddSpacer((10, 10))
		self.sbSizer1.Add(self.heading2,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer1.Add(self.sLine2,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer1.Add(self.btnAddWaypoint,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer1.Add(self.btnDelWaypoint,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT|wx.BOTTOM,10)
		
		#-----------------    Static Box for Robot Config
		self.staticBoxExec = wx.StaticBox(self, -2, 'Robot', size=(100, 100))
		self.btnConnect = wx.Button(self, -1, 'Connect Bot', (50, 50), (60, -1))
		self.btnLoadMap = wx.Button(self, -1, 'Load Onboard Map', (50, 50), (60, -1))
		self.btnInitial = wx.Button(self, -1, 'Reset Location', (50, 50), (60, -1))
		self.btnRun = wx.Button(self, -1, 'Run Transporter', (50, 50), (60, -1))
		self.btnEStop = wx.Button(self, -1, 'Emergency Stop', (20, 25))
		
		self.btnRun.Disable()
		self.btnEStop.Disable()
		self.heading3 = wx.StaticText(self, -1, 'Config', (130, 15))
		self.heading3.SetFont(self.font)
		self.sLine3 = wx.StaticLine(self, -1, (25, 50), (100,1))
		self.sLine4 = wx.StaticLine(self, -1, (25, 50), (100,1))
		self.heading4 = wx.StaticText(self, -1, 'Execute ', (130, 15))
		self.heading4.SetFont(self.font)
		
		self.staticBoxExec.SetFont(self.font)
		
		self.sbSizer2 = wx.StaticBoxSizer(self.staticBoxExec, wx.VERTICAL)
		self.sbSizer2.AddSpacer((10, 4))
		self.sbSizer2.Add(self.heading3,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer2.Add(self.sLine3,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer2.Add(self.btnConnect,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer2.Add(self.btnLoadMap,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer2.Add(self.btnInitial,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer2.AddSpacer((10, 4))
		self.sbSizer2.Add(self.heading4,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer2.Add(self.sLine4,0,wx.EXPAND|wx.TOP|wx.LEFT|wx.RIGHT,10)
		self.sbSizer2.Add(self.btnRun,1,wx.EXPAND|wx.ALL,10)
		self.sbSizer2.Add(self.btnEStop,2,wx.EXPAND|wx.ALL,10)
		
		#--------------------    Main Side Tab
		
		
		self.tabSizer.AddSpacer((10, 2))
		self.tabSizer.Add(self.sbSizer1,0,wx.EXPAND|wx.TOP|wx.RIGHT,10)
		self.tabSizer.AddSpacer((20, 20))
		self.tabSizer.Add(self.sbSizer2,0,wx.EXPAND|wx.TOP|wx.RIGHT,10)
		
		
		self.SetSizer(self.topSizer)
		self.SetAutoLayout(1)
		self.topSizer.Fit(self)
		self.SetMinSize((1000, 780))
		
		self.Centre()
		self.acquirer_thread = AcquirerThread(self)
		self.goal_thread = GoalThread(self)
		EVT_RESULT(self,self.OnResult)
		FIRST_TIME = 0

		self.Bind(wx.EVT_BUTTON, self.OnAddWP, self.btnAddWaypoint)
		self.Bind(wx.EVT_BUTTON, self.OnDelWP, self.btnDelWaypoint)
		self.Bind(wx.EVT_BUTTON, self.OnAddPath, self.btnNewPath)
		self.Bind(wx.EVT_BUTTON, self.OnResetLoc, self.btnInitial)
		
		self.Bind(wx.EVT_BUTTON, self.OnClearPath, self.btnClearPath)
		self.Bind(wx.EVT_BUTTON, self.OnUndoPath, self.btnUndoPath)
		
		self.Bind(wx.EVT_BUTTON, self.OnConnect, self.btnConnect)
		self.Bind(wx.EVT_BUTTON, self.OnLoadMap, self.btnLoadMap)
		self.Bind(wx.EVT_BUTTON, self.OnRun, self.btnRun)
		self.Bind(wx.EVT_BUTTON, self.OnEStop, self.btnEStop)
		
	
	def OnAddWP(self, event):
		global STATE_flag
		STATE_flag = STATE.ADD_WP
		self.statusBar.SetStatusText('Click anywhere on map to add waypoint')
		event.Skip()
		
		
	def OnDelWP(self, event):
		global STATE_flag
		STATE_flag = STATE.DEL_WP
		self.statusBar.SetStatusText('Click on a waypoint to delete')
		event.Skip()
		
	def OnAddPath(self, event):
		global STATE_flag
		STATE_flag = STATE.ADD_PATH
		self.statusBar.SetStatusText('Plan the path by clicking on waypoints')
		event.Skip()
		
	def OnResetLoc(self, event):
		global STATE_flag
		STATE_flag = STATE.RESET_LOC
		initialPose()
		self.statusBar.SetStatusText('Robot reset to home position:')
		event.Skip()
		dc = wx.BufferedDC(None, self.canvas.buffer)
		self.canvas.DoDrawing(dc,IMG_ORIGIN)
		self.canvas.Refresh()
	
	def OnClearPath(self, event):
		global PATH_cnt, PATH, ROBOT_POS
		self.statusBar.SetStatusText('Path cleared. Click on New Path to draw a new route.')
		PATH_cnt = 0
		PATH = []
		#PATH.append(ROBOT_POS)
		dc = wx.BufferedDC(None, self.canvas.buffer)
		self.canvas.DoDrawing(dc,IMG_ORIGIN)
		self.canvas.Refresh()
		
		event.Skip()
		
	def OnUndoPath(self, event):
		global PATH_cnt, PATH, ROBOT_POS
		if PATH_cnt > 0:
			PATH_cnt -= 1
			PATH.pop()
		#PATH.append(ROBOT_POS)
		dc = wx.BufferedDC(None, self.canvas.buffer)
		self.canvas.DoDrawing(dc,IMG_ORIGIN)
		self.canvas.Refresh()
		
		event.Skip()

	def OnResult(self, event):
		"""Show Result status."""
		# Process results here
		#self.statusBar.SetStatusText('Moving:')
		#print event.data
		if event.data[0] == 'complete':
			self.btnRun.Enable()
			self.btnEStop.Disable()
			return
		ROBOT_POS[0] = (math.fabs(MAP_ORIGIN[0]) + event.data[0])/MAP_RES 
		ROBOT_POS[1] = MAP_SIZE[1] - math.fabs(MAP_ORIGIN[1])/MAP_RES - event.data[1]/MAP_RES
		#print (event.data)
		#print (ROBOT_POS)
		#print (MAP_SIZE)
		#print MAP_ORIGIN
		#print (IMG_ORIGIN)
		
		dc = wx.BufferedDC(None, self.canvas.buffer)
		self.canvas.DoDrawing(dc,IMG_ORIGIN)
		self.canvas.Refresh()
		event.Skip()
		#self.canvas.OnClick(wx.EVT_LEFT_UP)
		
	def OnConnect(self, event):
		self.statusBar.SetStatusText('Connecting...')
		rospy.init_node('waypointGUI', anonymous=True)
		self.statusBar.SetStatusText('Connected. Node Initiated.')
		self.btnRun.Enable()
		
		
		global ROBOT_CONNECTED
		ROBOT_CONNECTED = 1
		dc = wx.BufferedDC(None, self.canvas.buffer)
		self.canvas.DoDrawing(dc,IMG_ORIGIN)
		self.canvas.Refresh()
		
	def OnLoadMap(self, event):
		self.statusBar.SetStatusText('Waiting for Map')
		rospy.wait_for_service('static_map')
		try:
			mapsrv = rospy.ServiceProxy('static_map', GetMap)
			resp = mapsrv()
			print resp.map.info
		except rospy.ServiceException, e:
			self.statusBar.SetStatusText('GetMap service call failed')
		self.btnConnect.Disable()
		self.statusBar.SetStatusText('Map Aquired. Waiting to save')
		
		#print resp.map.info.resolution
		#print resp.map.info.width
		#print resp.map.info.height
		
		global MAP_RES, MAP_SIZE, MAP_ORIGIN
		MAP_RES = resp.map.info.resolution
		MAP_SIZE[0] = resp.map.info.width
		MAP_SIZE[1] = resp.map.info.height
		MAP_ORIGIN = [resp.map.info.origin.position.x,resp.map.info.origin.position.y]
		#print MAP_ORIGIN
		
		x=0
		y=0
		
		self.wxImg = wx.EmptyImage( resp.map.info.width, resp.map.info.height )
		
		while y < resp.map.info.height:
			x = 0
			while x < resp.map.info.width:
				idx = x + (resp.map.info.height - y - 1) * resp.map.info.width
				if resp.map.data[idx] == 0:
					#print'0'
					self.wxImg.SetRGB(x,y,254,254,254)
				elif resp.map.data[idx] == 100:
					#print '100'
					self.wxImg.SetRGB(x,y,0,0,0)
				else:
					#print resp.map.data[idx]
					self.wxImg.SetRGB(x,y,205,205,205)
					
				x += 1	
			y += 1
			
	
	
		self.pilImage = Image.new( 'RGB', (self.wxImg.GetWidth(), self.wxImg.GetHeight()) )
		self.pilImage.fromstring( self.wxImg.GetData() )
		self.pilImage.save(icons_path + "mapFromRobot.png")
		self.canvas.UpdateImages(icons_path + "mapFromRobot.png")
		self.statusBar.SetStatusText('Map Aquired and Saved...')
		
		
	def OnRun(self, event):
		global ESTOP
		if PATH_cnt == 0:
			self.statusBar.SetStatusText('No Path given... Create a new path connecting waypoints before Run')
		else:
			self.statusBar.SetStatusText('Running current path...')
			self.btnRun.Disable()
			self.btnEStop.Enable()
			global GOAL_SET, GOAL_REACHED
			ESTOP = False
			eStop(ESTOP)
			GOAL_SET = 1
			GOAL_REACHED = 0
		
	def OnEStop(self, event):
		self.statusBar.SetStatusText('Transporter Paused. Click Run to resume...')
		self.btnRun.Enable()
		self.btnEStop.Disable()
		global ESTOP
		ESTOP = True
		eStop(ESTOP)
	
	
class MainApp(wx.App):
    """Class Main App."""
    def OnInit(self):
        """Init Main App."""
        self.frame = navWindow(None, -1, 'turtle Navigator')
        self.frame.Show(True)
        self.SetTopWindow(self.frame)
        return True

if __name__ == '__main__':
	BUFFERED = 1
	FIRST_TIME = 0

	
	icons_path = path.join(roslib.packages.get_pkg_dir('tbot_control'), "images/")
    	app = MainApp(0)
    	app.MainLoop()

	



	
