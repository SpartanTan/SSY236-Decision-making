:- discontiguous drink/1.

softdrink(cola1).
softdrink(cola2).
softdrink(fanta1).
softdrink(fanta2).

drink(milk1).
food(butter1).

storage_place(cupboard1).
storage_place(fridge1).

in_place(butter1, fridge1).
in_place(milk1, fridge1).
in_place(cola2, fridge1).
in_place(cola1, cupboard1).
in_place(fanta2, cupboard1).

drink(X) :- softdrink(X).
is_drinkable(X,Y):- drink(X), in_place(X,Y).

mark_likes(X) :- is_drinkable(X,Y), Y='fridge1'.

robot(tiago).
next_to(tiago, fridge1).

can_grasp(X):- is_drinkable(X,Y), robot(Z), next_to(Z,Y).