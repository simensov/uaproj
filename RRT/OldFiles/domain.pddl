
(define (domain blocksworld)
(:predicates (clear ?x)
             (on ?x ?y)
             (on-table ?b)
             (holding ?b)
             (arm-free))

(:action stack
  :parameters (?a ?b)
  :precondition (and (clear ?b) (holding ?a) )
  :effect (and (not (clear ?b)) (not (holding ?a))
               (on ?a ?b) (clear ?a) (arm-free)))
 
(:action pick-from-block
  :parameters (?a ?b)
  :precondition (and (clear ?a) (arm-free) (on ?a ?b))
  :effect (and (not (clear ?a)) (not (arm-free)) (holding ?a)
               (not (on ?a ?b)) (clear ?b)
          ))

(:action pick-from-table
  :parameters (?a)
  :precondition (and (clear ?a) (arm-free) (on-table ?a))
  :effect (and (not (clear ?a)) (not (arm-free)) (holding ?a)
               (not (on-table ?a))
          ))

(:action put-on-table
  :parameters (?b)
  :precondition (and (holding ?b))
  :effect (and (clear ?b) (arm-free) (on-table ?b)
          (not (holding ?b)))
          ))
 