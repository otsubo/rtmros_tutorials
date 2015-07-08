(load "package://hrpsys_gazebo_tutorials/euslisp/samplerobot-interface-ex.l")
(load "package://drc_task_common/euslisp/robot-util.l")

(init)
;;(reset-manip-pose)
;;(model2real)
(ros::roseus "samplerobot-reaching")
(setq *robot-name* "Samplerobot")


(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init))
  )

(setq *target-coords* (make-coords))
(setq *finish-flag* nil)

(defun subscribe_valve (msg)
  (let ((trans
         (send *tfl* :lookup-transform
               "/WAIST_LINK0"
               (send msg :header :frame_id)
               (ros::time 0))))
    (setq *target-coords* (send (send trans :copy-worldcoords) :transform (ros::tf-pose->coords (send msg :pose))))
    (ros::unsubscribe "centroid_publisher/output")
    (setq *finish-flag* t)
    ;;(reaching-motion)
    )
  )

(defun test-reaching 
  ()
  (ros::subscribe "centroid_publisher/output/pose" geometry_msgs::PoseStamped #'subscribe_valve)
  (ros::rate 10)
  (warn "please click the drill")
  (while (and (ros::ok) (not *finish-flag*))
    (ros::spin-once)
    )
  (print *target-coords*)
  (generate-reaching-motion)
  )

(defun walk-to-target
  ()
  (setq *stand-point* (v- (send (send *target-coords* :copy-worldcoords) :pos) (float-vector 500 100 0)))
  (send *ri* :go-pos (* (elt *stand-point* 0) 0.001) (* (elt *stand-point* 1) 0.001) 0)
)

(defun generate-reaching-motion 
  ()
  (setq *target-coords-for-ik* (make-coords :pos (float-vector (- (elt (send *target-coords* :pos) 0) 500) (elt (send *target-coords* :pos) 1) 1171)))
  (let* ((tc
          (list 
           (send *target-coords-for-ik* :copy-worldcoords)
           (send *robot* :rleg :end-coords :copy-worldcoords)
           (send *robot* :lleg :end-coords :copy-worldcoords))))
    (with-move-target-link-list
     (mt ll *robot* '(:rarm :rleg :lleg))
     (send *robot*
           :fullbody-inverse-kinematics tc
           :move-target mt :link-list ll
           :root-link-virtual-joint-weight #f(0.1 0.1 0.1 0.1 0.5 0.5)
           :target-centroid-pos (apply #'midpoint 0.5 (send *robot* :legs :end-coords :worldpos))
           :translation-axis (list t t t)
           :rotation-axis (list :z t t)
           :stop 500
           )
     (send *irtviewer* :draw-objects :flush nil)
     ))
  )

(defun main
  ()
  (send *ri* :calibrate-inertia-sensor)
  (unix::sleep 5)
  (reaching-pose)
  (send *ri* :start-auto-balancer)
  (send *ri* :start-st)
  (send *ri* :start-impedance :arms)
  (send *ri* :go-pos 0.2 0 0)
  (grasp)
  (reaching-pose2)
  (send *ri* :go-pos 0 0.3 0)
  (send *ri* :go-pos 0 0.3 0)
  (send *ri* :go-pos 0 0.4 0)
  (send *ri* :stop-st)
  (reaching-pose)
  )

(defun reset
  ()
  (send *ri* :stop-auto-balancer)
  (send *ri* :stop-st)
  (send *ri* :angle-vector (send *robot* :init-pose) 3000)
  (setq *target-coords* (make-coords))
  (setq *finish-flag* nil)
  )

(defun grasp
  ()
  (send *robot* :angle-vector #f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 0.0 0.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 0.0 0.0 -100.0 -19.0 -6.5 0.0 0.0 0.0 0.0))
  (model2real)
  )

(defun grasp-pose
  ()
  (send *robot* :angle-vector #f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 10.0 -10.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -10.0 10.0 -100.0 -9.0 -6.5 0.0 0.0 0.0 0.0))
  (model2real)
  )


(defun reaching-pose
  ()
  (send *robot* :angle-vector #f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -50.0 -10.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 50.0 10.0 -100.0 -9.0 -6.5 0.0 0.0 0.0 0.0))
  (model2real)
  )


#f(0.0 -20.0 0.0 47.0 -27.0 0.0 -71.48 -1.58201 10.0 -60.0 15.6772 50.0 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 -71.48 1.58201 -10.0 -60.0 -15.6772 50.0 0.0 0.0 0.0 0.0)


#f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -20.0 -10.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 20.0 10.0 -100.0 -9.0 -6.5 0.0 0.0 0.0 0.0)

#f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 10.0 -10.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -10.0 10.0 -100.0 -9.0 -6.5 0.0 0.0 0.0 0.0)

#f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 10.0 -10.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -10.0 10.0 -100.0 -9.0 -6.5 0.0 0.0 0.0 0.0)

#f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -20.0 5.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 20.0 -5.0 -100.0 -9.0 -6.5 0.0 0.0 0.0 0.0)


#f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -50.0 -10.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 50.0 10.0 -100.0 -9.0 -6.5 0.0 0.0 0.0 0.0)
#f(0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 -2.5 -10.0 -100.0 9.0 -6.5 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 30.0 2.5 10.0 -100.0 -9.0 -6.5 0.0 0.0 0.0 0.0)


(defun reaching-pose2
  ()
  (send *robot* :angle-vector #f(0.0 -20.0 0.0 47.0 -27.0 0.0 12.6459 6.48292 -2.07462 -135.918 12.7801 47.7544 0.0 0.0 -20.0 0.0 47.0 -27.0 0.0 12.6459 -6.48292 2.07462 -135.918 -12.7801 47.7544 0.0 0.0 0.0 0.0))
  (model2real)
  )

