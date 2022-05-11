# Note for the teams
Make a separate branch for your work. Agree on a naming convention and stick with it.

The following is the controls team naming convention (put one for safety team as well):

# Naming convention for programs:
naming the file is as follows: function of the program (i.e. tracker) (for the top level python file, use underscore MAIN (i.e. _MAIN)) +
drone number (if applicable) in cf# format  (i.e. cf3)

# To commit and push to a new branch
- git checkout -b [branch-name] where [branch name includes group and date]
- use mm_dd_yy format for date
- git commit -am "[message]" 
- git push ... pushes to CooperDrones github

Controls team branch naming convention:
Controls_mm_dd_yy (Ex: Controls_05_04_22)

# Github Account
Username: CooperDrones
Password: DLuchtenburg705
Personal access token (use this when git push prompts for a password): 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

Note: When pushing, copy and paste the password using right click. Do not use shortkeys.


# DEBUGGING 
For showing the trees of the ROS msg, rqt_graph
For showing the tree of Vicon, rosrun rqt_tf_tree rqt_tf_tree 

