/*
Figure 4.1(Right)
*/
map(c4,pit).
map(c8,gold).
map(c11,wumpus).
map(c9, pit).
map(c13,pit).


adj_of(c1,c2).
adj_of(c1,c5).
adj_of(c2,c3).
adj_of(c2,c6).
adj_of(c3,c4).
adj_of(c3,c7).
adj_of(c4,c8).
adj_of(c5,c6).
adj_of(c5,c9).
adj_of(c6,c7).
adj_of(c6,c10).
adj_of(c7,c11).
adj_of(c7,c8).
adj_of(c8,c12).
adj_of(c9,c13).
adj_of(c9,c10).
adj_of(c10,c14).
adj_of(c10,c11).
adj_of(c11,c15).
adj_of(c11,c12).
adj_of(c12,c16).

g(X) :- map(X,gold).

avai_moves(X,Y) :- (adj_of(X,Y);adj_of(Y,X)).
is_safe(Y) :- not(map(Y,pit)),not(map(Y,wumpus)).

safe_moves(X,Y) :-avai_moves(X,Y), is_safe(Y).

move(X,InitialList, ActionList, MaxActions):- 
    safe_moves(X,Y),
    append(InitialList,[Y], I),
   	N is MaxActions-1,
    nat(N),
    move(Y,I,ActionList,N). 

test(Action) :- write(Action), N is Action-1,nat(N),test(N).
nat(N) :- N > 0.

move2(X,InitialList, ActionList, MaxActions):- 
    g(X),
    ActionList = InitialList;
    safe_moves(X,Y),
    not(member(Y,InitialList)),
    append(InitialList,[Y], L),
    N is MaxActions-1,
    nat(N),
    move2(Y, L, ActionList, N).



