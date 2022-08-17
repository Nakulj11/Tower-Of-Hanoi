 
(define (domain TOWER-OF-HANOI)
    (:requirements :strips :typing :negative-preconditions :disjunctive-preconditions)
    (:types disk tower)
 
    (:predicates (topoftower ?x - disk ?y - tower)
                 (ondisk ?x - disk ?y - disk ?z - tower)
                 (issmaller ?x - disk ?y - disk)
                 (handempty)
                 (holding ?x - disk)
                 (emptytower ?x - tower)
          (bottomoftower ?x - disk ?y - tower)
                 )
 
    (:action pick-up-1
          :parameters (?x - disk ?y - tower)
          :precondition
         (and (topoftower ?x ?y)
              (handempty)
              (bottomoftower ?x ?y))
          :effect
          (and (not (topoftower ?x ?y))
             (not (handempty))
             (holding ?x)
             (not (bottomoftower ?x ?y))
             (emptytower ?y))
    )
 
    (:action pick-up-from-stack-2
         :parameters (?x - disk ?y - disk ?z - tower)
          :precondition
         (and (topoftower ?x ?z)
              (handempty)
              (ondisk ?x ?y ?z))
          :effect
          (and (not (topoftower ?x ?z))
               (topoftower ?y ?z)
               (not (ondisk ?x ?y ?z))
               (not (handempty))
               (holding ?x))
    )
 
    (:action place-on-tower-3
         :parameters (?x - disk ?y - tower)
         :precondition
         (and (holding ?x)
              (emptytower ?y))
         :effect
         (and (not (holding ?x))
             (topoftower ?x ?y)
             (handempty)
             (not (emptytower ?y))
             (bottomoftower ?x ?y))
    )
 
    (:action place-on-stack-4
         :parameters (?x - disk ?y - disk ?z - tower)
         :precondition
         (and (holding ?x)
              (topoftower ?y ?z)
              (issmaller ?x ?y))
         :effect
         (and (not (holding ?x))
             (not (topoftower ?y ?z))
             (ondisk ?x ?y ?z)
             (handempty)
             (topoftower ?x ?z))
    )
    
)
