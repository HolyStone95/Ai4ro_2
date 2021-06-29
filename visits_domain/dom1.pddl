(define (domain localization)

(:requirements :typing :durative-actions :numeric-fluents :negative-preconditions :action-costs :conditional-effects :equality :fluents )


(:types 	robot region 
)

(:predicates
		(robot_in ?v - robot ?r - region) 
		(visited ?r - region )
		(moved ?from ?to - region)
		(localized)
	      
)

(:functions 
		(act-cost) 
		(triggered ?from ?to - region) 
		(dummy)
)

(:durative-action goto_region
		:parameters (?v - robot ?from ?to - region)
		:duration (= ?duration 100)
		:condition (and (at start (robot_in ?v ?from))(at start (localized)))
	        :effect (and (at start (not (robot_in ?v ?from))) (at start (not(localized))) (at start (increase (triggered ?from ?to) 1))
		(at end (robot_in ?v ?to)) (at end (assign (triggered ?from ?to) 0)) (at end (visited ?to))(at end (moved ?from ?to)))
)


(:durative-action localize
		:parameters (?v - robot ?from ?to - region)
		:duration (= ?duration 10)
		:condition (and (at start (visited ?to))(at start (moved ?from ?to)))
	        :effect (and (at end (increase (act-cost) (dummy)))(at end (localized))(at end (not (moved ?from ?to)))))
)

