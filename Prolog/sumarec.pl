sum(0,0).
sum(N,S):- nat(N), K is N-1, sum(K,T), S is N+T.
nat(X):- integer(X), X>=0.
