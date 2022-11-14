write_star(0) :- !.
write_star(N):- nat(N-1), T is N-1, write('#'), write_star(T).

write_space(0) :- !.
write_space(N):- nat(N-1), T is N-1, write(' '), write_space(T).

nat(T) :- T >=0.

up(_,0):- !.
%process(N):- first, nl, second, nl, third.
up(N,L):- T is L-1,K is N-T, draw_up(T,K), up(N,T).
draw_up(T,K) :- write_space(T), write_star(K),nl.

down(_,0):- !.
%process(N):- first, nl, second, nl, third.
down(N,L):- T is N-L, K is N-T, P is L-1, draw_down(T,K), down(N,P).
draw_down(T,K) :- write_space(T), write_star(K),nl.

all(N) :- up(N,N), down(N,N-1).