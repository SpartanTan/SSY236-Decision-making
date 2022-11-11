write_star(0) :- !.
write_star(N):- nat(N-1), T is N-1, write('#'), write_star(T).

write_space(0) :- !.
write_space(N):- nat(N-1), T is N-1, write(' '), write_space(T).

nat(T) :- T >=0.
process(N):- first, nl, second, nl, third.
first :- write_space(3), write_star(1).
second:- write_space(2), write_star(2).
third :- write_space(1), write_star(3).