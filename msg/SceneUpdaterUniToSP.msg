string got_object_action  	# add, moveto, remove, attach, detach
string got_object_name    	# object name
float32[] got_euler_pose  	# euler pose for the object
string[] attached_objects		# list of names of the attached objects ('none' if None)
PsiObjectPose[] object_poses	# returns the list of objects in the scene and their poses
string error