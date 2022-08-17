(define (problem PROBLEM-OF-HANOI-3)
(:domain TOWER-OF-HANOI)
(:objects A B C - disk T1 T2 T3 - tower)
(:INIT (HANDEMPTY)
       (ISSMALLER A B) (ISSMALLER A C) (ISSMALLER B C)
       


(EMPTYTOWER T1) (EMPTYTOWER T2) (TOPOFTOWER B T3) (ONDISK B A T3) (BOTTOMOFTOWER A T3) 





       
       )
(:goal (AND (ONDISK A B T3) (ONDISK B C T3)))
)
