### Report 25/02/16

Modifiche fatte sino ad oggi:
* introdotto delay tra un polling di un beacon e il successivo _peggioramento signigicativo della posizione calcolata rispetto al modello ideale, aumento consistente del DistanceErrorAvg_
 * DistanceErrorAvg 2.6528 img/polling_delay.jpg

* introdotta scelta dei beacon migliori da cui fare polling basandosi sulla posizione dedotta al tempo t-1 _miglioramento della posizione rilevata e decremento del DistanceErrorAvg rispetto al caso precedente_
 * A causa dell'imperfetta copertura del percorso l'errore risulta ancora elevato
 * DistanceErrorAvg 2.24 'img/best_beacon_selection.jpg'
 
* modifica posizione beacon per migliorare la copertura del percorso _significativo miglioramento della posizione rilevata e decremento del DistanceErrorAvg_
 * DistanceErrorAvg 0.96 'img/beacon_new_position.jpg'

### Report 16/03/16

* aggiunta calcolo della distanza pesata mediante l'uso della matrice delle coovarianze _quasi impercettibile peggioramente del tracking della posizione_
 * DistanceErrorAvg 1.22 vs 1.20 derivato dal calcolo della distanza euclidea

* aggiunti muri dimostrativi nel plot, è opzionalmente possibile evitare di mostrare il range dei beacon

### Report 6/04/16

* aggiunta dati precedentemente ottenuti e relativo fitting con varie curve _è risultato ottimo il fitting con curve polinomiali di terzo grado_

* modifica della funzione per il calcolo del valore RSSI nel modello di matlab, viene utilizzata la funzione descritta nel file "space-to-rssi-poly3.sfit"
 * Si ha un peggioramento del valore DistanceErrorAvg 1.5 contro il precedente 1.2
