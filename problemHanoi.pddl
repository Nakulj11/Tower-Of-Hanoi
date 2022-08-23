(define (problem PROBLEM-OF-HANOI-3)
(:domain TOWER-OF-HANOI)
(:objects A B C - disk T1 T2 T3 - tower)
(:INIT (HANDEMPTY)
       (ISSMALLER A B) (ISSMALLER A C) (ISSMALLER B C)
       


(TOPOFTOWER a t1) (ONDISK a c t1) (BOTTOMOFTOWER c t1) (EMPTYTOWER t2) (TOPOFTOWER b t3) (BOTTOMOFTOWER b t3) 





       
       )
(:goal (AND (ONDISK A B T3) (ONDISK B C T3)))
)
