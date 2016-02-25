### Report 25/02/16

Modifiche fatte sino ad oggi:
* introdotto delay tra un polling di un beacon e il successivo _peggioramento signigicativo della posizione calcolata rispetto al modello ideale, aumento consistente del DistanceErrorAvg_
 * DistanceErrorAvg 2.6528 img/polling_delay.jpg

* introdotta scelta dei beacon migliori da cui fare polling basandosi sulla posizione dedotta al tempo t-1 _miglioramento della posizione rilevata e decremento del DistanceErrorAvg rispetto al caso precendete_
 * A causa dell'imperfetta copertura del percorso l'errore risulta ancora elevato
 * DistanceErrorAvg 2.24 'img/best_beacon_selection.jpg'
 
* modifica posizione beacon per migliorare la copertura del percorso _significativo miglioramento della posizione rilevata e decremento del DistanceErrorAvg_
 * DistanceErrorAvg 0.96 'img/beacon_new_position.jpg'

