
(define (problem wrods)
    (:domain blocksworld)
    (:objects A B C D E F G)
    (:init
     (clear E)(on E A)(on-table A)
     (clear B)(on B C)(on C D)(on-table D)
      (arm-free))
    (:goal (and (clear A)(on A B)(on B C)(on C D)(on D E)(on-table E)))
           
    )