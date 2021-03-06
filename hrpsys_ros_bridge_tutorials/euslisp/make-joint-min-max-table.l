
(defun gen-min-max-table-hrp2
  (robot margin)
  "Generate min-max table for hrp2 robots.
   robot is robot instance.
   margin is joint limit margin [deg]."
  (dolist (limb '(:rarm :larm))
    (if (send robot limb :wrist-r)
        (send robot :make-joint-min-max-table
              (send robot limb :wrist-r :parent-link)
              (send robot limb :wrist-p :child-link)
              (send robot limb :wrist-r)
              (send robot limb :wrist-p)
              :margin margin)))
  (dolist (limb '(:rleg :lleg))
    (if (send robot limb :crotch-r)
        (send robot :make-joint-min-max-table
              (send robot limb :crotch-r :parent-link)
              (send robot limb :crotch-p :child-link)
              (send robot limb :crotch-r)
              (send robot limb :crotch-p)
              :margin margin)))
  (if (and (send robot :rleg :crotch-y) (send robot :lleg :crotch-y))
      (send robot :make-joint-min-max-table
            (send robot :rleg :crotch-y :child-link)
            (send robot :lleg :crotch-y :child-link)
            (send robot :rleg :crotch-y)
            (send robot :lleg :crotch-y)
            :margin margin))
  (if (and (send robot :torso :waist-y) (send robot :torso :waist-p))
      (send robot :make-joint-min-max-table
            (send robot :torso :waist-y :parent-link)
            (send robot :torso :waist-p :child-link)
            (send robot :torso :waist-y)
            (send robot :torso :waist-p)
            :margin margin))
  ;; Hand made ankle-r ankle-p min-max table measured by real robot encoder values.
  ;;  This is for kawasaki-san ankle spacer.
  (when (string= (send robot :name) "HRP2JSK")
    ;; set org-min-angle and org-max-angle for debug
    (send (send robot :joint "RLEG_JOINT5") :put :org-min-angle -29.0)
    (send (send robot :joint "RLEG_JOINT5") :put :org-max-angle 45.0)
    (send (send robot :joint "LLEG_JOINT5") :put :org-min-angle -45.0)
    (send (send robot :joint "LLEG_JOINT5") :put :org-max-angle 29.0)
    (send (send robot :joint "RLEG_JOINT4") :put :org-min-angle -74.0)
    (send (send robot :joint "RLEG_JOINT4") :put :org-max-angle 42.0)
    (send (send robot :joint "LLEG_JOINT4") :put :org-min-angle -74.0)
    (send (send robot :joint "LLEG_JOINT4") :put :org-max-angle 42.0)
    ;; set joint mm tables
    (send (send robot :joint "RLEG_JOINT5") :joint-min-max-table
          '#2f((-74.0 -73.0 -72.0 -71.0 -70.0 -69.0 -68.0 -67.0 -66.0 -65.0 -64.0 -63.0 -62.0 -61.0 -60.0 -59.0 -58.0 -57.0 -56.0 -55.0 -54.0 -53.0 -52.0 -51.0 -50.0 -49.0 -48.0 -47.0 -46.0 -45.0 -44.0 -43.0 -42.0 -41.0 -40.0 -39.0 -38.0 -37.0 -36.0 -35.0 -34.0 -33.0 -32.0 -31.0 -30.0 -29.0 -28.0 -27.0 -26.0 -25.0 -24.0 -23.0 -22.0 -21.0 -20.0 -19.0 -18.0 -17.0 -16.0 -15.0 -14.0 -13.0 -12.0 -11.0 -10.0 -9.0 -8.0 -7.0 -6.0 -5.0 -4.0 -3.0 -2.0 -1.0 0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 31.0 32.0 33.0 34.0 35.0 36.0 37.0 38.0 39.0)
               (-19.0 -19.0 -20.0 -21.0 -21.0 -22.0 -23.0 -23.0 -24.0 -24.0 -25.0 -25.0 -25.0 -26.0 -26.0 -27.0 -27.0 -27.0 -27.0 -27.0 -28.0 -28.0 -28.0 -28.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0 -29.0)
               (22.0 22.0 23.0 23.0 24.0 24.0 25.0 25.0 26.0 27.0 28.0 28.0 29.0 29.0 30.0 31.0 31.0 32.0 32.0 33.0 34.0 34.0 35.0 36.0 37.0 37.0 38.0 39.0 40.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 41.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 42.0 41.0 40.0 40.0 39.0 39.0 39.0 40.0 41.0 42.0 43.0 43.0 43.0 43.0 43.0 43.0 42.0 41.0 41.0 40.0 40.0 39.0 38.0 37.0 36.0 36.0 35.0 34.0 33.0 32.0 31.0 30.0 29.0 29.0 28.0 28.0 27.0 27.0 26.0 26.0))
          )
    (send (send robot :joint "RLEG_JOINT5") :joint-min-max-target (send robot :joint "RLEG_JOINT4"))
    (send (send robot :joint "RLEG_JOINT4") :joint-min-max-table
          '#2f((-30.0 -29.0 -28.0 -27.0 -26.0 -25.0 -24.0 -23.0 -22.0 -21.0 -20.0 -19.0 -18.0 -17.0 -16.0 -15.0 -14.0 -13.0 -12.0 -11.0 -10.0 -9.0 -8.0 -7.0 -6.0 -5.0 -4.0 -3.0 -2.0 -1.0 0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 31.0 32.0 33.0 34.0 35.0 36.0 37.0 38.0 39.0 40.0 41.0 42.0 43.0 44.0)
               (-49.0 -53.0 -58.0 -60.0 -63.0 -65.0 -67.0 -68.0 -70.0 -71.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -71.0 -69.0 -67.0 -65.0 -64.0 -63.0 -61.0 -59.0 -58.0 -56.0 -54.0 -53.0 -51.0 -50.0 -49.0 -47.0 -46.0 -45.0 -44.0 -24.0 11.0)
               (38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 36.0 34.0 32.0 30.0 29.0 28.0 27.0 26.0 25.0 24.0 22.0 21.0 20.0 19.0 17.0 15.0 14.0))
          )
    (send (send robot :joint "RLEG_JOINT4") :joint-min-max-target (send robot :joint "RLEG_JOINT5"))
    (send (send robot :joint "LLEG_JOINT5") :joint-min-max-table
          '#2f((-74.0 -73.0 -72.0 -71.0 -70.0 -69.0 -68.0 -67.0 -66.0 -65.0 -64.0 -63.0 -62.0 -61.0 -60.0 -59.0 -58.0 -57.0 -56.0 -55.0 -54.0 -53.0 -52.0 -51.0 -50.0 -49.0 -48.0 -47.0 -46.0 -45.0 -44.0 -43.0 -42.0 -41.0 -40.0 -39.0 -38.0 -37.0 -36.0 -35.0 -34.0 -33.0 -32.0 -31.0 -30.0 -29.0 -28.0 -27.0 -26.0 -25.0 -24.0 -23.0 -22.0 -21.0 -20.0 -19.0 -18.0 -17.0 -16.0 -15.0 -14.0 -13.0 -12.0 -11.0 -10.0 -9.0 -8.0 -7.0 -6.0 -5.0 -4.0 -3.0 -2.0 -1.0 0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 31.0 32.0 33.0 34.0 35.0 36.0 37.0 38.0 39.0)
               (-22.0 -22.0 -23.0 -23.0 -24.0 -24.0 -25.0 -25.0 -26.0 -27.0 -28.0 -28.0 -29.0 -29.0 -30.0 -31.0 -31.0 -32.0 -32.0 -33.0 -34.0 -34.0 -35.0 -36.0 -37.0 -37.0 -38.0 -39.0 -40.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -41.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -42.0 -41.0 -40.0 -40.0 -39.0 -39.0 -39.0 -40.0 -41.0 -42.0 -43.0 -43.0 -43.0 -43.0 -43.0 -43.0 -42.0 -41.0 -41.0 -40.0 -40.0 -39.0 -38.0 -37.0 -36.0 -36.0 -35.0 -34.0 -33.0 -32.0 -31.0 -30.0 -29.0 -29.0 -28.0 -28.0 -27.0 -27.0 -26.0 -26.0)
               (19.0 19.0 20.0 21.0 21.0 22.0 23.0 23.0 24.0 24.0 25.0 25.0 25.0 26.0 26.0 27.0 27.0 27.0 27.0 27.0 28.0 28.0 28.0 28.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0 29.0))
          )
    (send (send robot :joint "LLEG_JOINT5") :joint-min-max-target (send robot :joint "LLEG_JOINT4"))
    (send (send robot :joint "LLEG_JOINT4") :joint-min-max-table
          '#2f((-44.0 -43.0 -42.0 -41.0 -40.0 -39.0 -38.0 -37.0 -36.0 -35.0 -34.0 -33.0 -32.0 -31.0 -30.0 -29.0 -28.0 -27.0 -26.0 -25.0 -24.0 -23.0 -22.0 -21.0 -20.0 -19.0 -18.0 -17.0 -16.0 -15.0 -14.0 -13.0 -12.0 -11.0 -10.0 -9.0 -8.0 -7.0 -6.0 -5.0 -4.0 -3.0 -2.0 -1.0 0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0)
               (11.0 -24.0 -44.0 -45.0 -46.0 -47.0 -49.0 -50.0 -51.0 -53.0 -54.0 -56.0 -58.0 -59.0 -61.0 -63.0 -64.0 -65.0 -67.0 -69.0 -71.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -73.0 -71.0 -70.0 -68.0 -67.0 -65.0 -63.0 -60.0 -58.0 -53.0 -49.0)
               (14.0 15.0 17.0 19.0 20.0 21.0 22.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 32.0 34.0 36.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0 38.0))
          )
    (send (send robot :joint "LLEG_JOINT4") :joint-min-max-target (send robot :joint "LLEG_JOINT5"))
    )
  )
(defun gen-min-max-table-jaxon
  (robot margin)
  "Generate min-max table for jaxon robots.
   robot is robot instance.
   margin is joint limit margin [deg]."
  (dolist (limb '(:rarm :larm))
    (send robot :make-joint-min-max-table
          (send robot limb :wrist-r :parent-link)
          (send robot limb :wrist-p :child-link)
          (send robot limb :wrist-r)
          (send robot limb :wrist-p)
          :margin margin))
  (if (and (send robot :torso :waist-r) (send robot :torso :waist-p))
      (send robot :make-joint-min-max-table
            (send robot :torso :waist-r :parent-link)
            (send robot :torso :waist-p :child-link)
            (send robot :torso :waist-r)
            (send robot :torso :waist-p)
            :margin 3))
  )
(defun gen-min-max-table-staro
  (robot margin)
  "Generate min-max table for staro robots.
   robot is robot instance.
   margin is joint limit margin [deg]."
  (send robot :make-joint-min-max-table
        (send robot :torso :waist-p :parent-link)
        (send robot :torso :waist-y :child-link)
        (send robot :torso :waist-p)
        (send robot :torso :waist-y)
        :margin margin)
  )

(defun write-min-max-table-to-robot-model-file
  (robot path &key (margin 1.0) (gen-min-max-table-func))
  "Generate and write min-max to robot model file.
   robot is robot instance.
   path is robot model file full path.
   margin is joint limit margin [deg].
   gen-min-max-table-func is function to generate min-max table for each robot."
  (let ((robot-name (send robot :name)))
    (cond ((substringp "hrp2" robot-name) (setq gen-min-max-table-func 'gen-min-max-table-hrp2))
          ((substringp "jaxon" robot-name) (setq gen-min-max-table-func 'gen-min-max-table-jaxon))
          ((substringp "staro" robot-name) (setq gen-min-max-table-func 'gen-min-max-table-staro))
          ))
  (when (remove-if-not #'(lambda (j) (send j :joint-min-max-table)) (send robot :joint-list))
    (warn ";; ~A already has min-max table.~%" (send robot :name))
    (return-from write-min-max-table-to-robot-model-file nil))
  ;; (gen-min-max-table robot (format nil "~Ayaml" (string-right-trim "l" path)))
  (funcall gen-min-max-table-func robot margin)
  (with-open-file
   (f path :direction :output :if-exists :append)
   (format f "(defmethod ~A~%" (send (class robot) :name))
   (format f "  (:init-ending~%")
   (format f "    ()~%")
   (format f "    (prog1~%")
   (format f "      (send-super :init-ending)~%")
   (labels
       ((gen-string-name
         (j)
         (if (stringp (send j :name)) (format nil "\"~A\"" (send j :name)) (send j :name))))
     (dolist (j (remove-if-not #'(lambda (j) (send j :joint-min-max-table)) (send robot :joint-list)))
       (format f "      (send (send self :joint ~A) :joint-min-max-table~%" (gen-string-name j))
       (format f "                              '~A~%" (send j :joint-min-max-table))
       (format f "                              )~%")
       (format f "      (send (send self :joint ~A) :joint-min-max-target (send self :joint ~A))~%"
               (gen-string-name j) (gen-string-name (send j :joint-min-max-target)))
       )
     (format f "      )))~%")
     ))
  t)

(defun write-min-max-table-to-conf-file
  (robot conf-file-path)
  (unless (car (piped-fork-returns-list (format nil "grep joint_limit_table ~A" conf-file-path)))
    (with-open-file
     (of conf-file-path :direction :output :if-exists :append)
     (let ((jmmtl (remove-if-not #'(lambda (x) (send x :joint-min-max-table)) (send robot :joint-list))))
       (when jmmtl
         (format of "~%~%# for joint limit table~%")
         (format of "joint_limit_table: ")
         (dolist (j jmmtl)
           (format of "~A:~A:" (send j :name) (send (send j :joint-min-max-target) :name)) ;; self_joint_name:target_joint_name
           (let* ((mmt (send j :joint-min-max-table))
                  (dims (array-dimensions mmt))
                  (str ""))
             (dotimes (i (car dims))
               (let ((str2 ""))
                 (if (= i 0)
                     (setq str2 (format nil "~A:~A" (round (aref mmt 0 0)) (round (aref mmt 0 (- (cadr (array-dimensions mmt)) 1))))) ;; target_min_angle:target_max_angle
                   (dotimes (j (cadr dims))
                     (setq str2 (if (= j 0) (format nil "~A" (round (aref mmt i j))) (format nil "~A,~A" str2 (round (aref mmt i j))))))) ;; min_table:max_table
                 (setq str (if (= i 0) (format nil "~A" str2) (format nil "~A:~A" str str2)))))
             (format of "~A:" str)
             ))))
     (format of "~%")
     )))

#|
;; How to check robot min-max table

(load "models/hrp2jsk.l")
(defmethod hrp2jsk-robot (:init-ending () (send-super :init-ending))) ;; Clear current min max table
(setq *robot* (hrp2jsk))
(send *robot* :make-joint-min-max-table
            (send *robot* :torso :waist-y :parent-link)
            (send *robot* :torso :waist-p :child-link)
            (send *robot* :torso :waist-y)
            (send *robot* :torso :waist-p)
            :margin 1.0)
(send *robot* :plot-joint-min-max-table (send *robot* :torso :waist-y) (send *robot* :torso :waist-p))
|#
