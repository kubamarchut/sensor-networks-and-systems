#include <Arduino.h>
#include "morslib.h"

morslib mymors(LED_BUILTIN, 200);//numer pinu, czas trwania kropki [ms]

void setup()
{
  mymors.begin();

  delay(2000);

  mymors.queue('p', 0);// znak do wyświeltenia moresem, (opcjonalnie, domyślnie 0) priorytet: 
  mymors.queue('a', 1);// 0 - znak zostanie dodany jako ostatni w kolejce do wyświetlenia
  mymors.queue('w', 0);// 1 - znak zostanie dodany jako następny w kolejce do wyświetlenia
  mymors.queue('e', 0);// lista dostępnych do wyswietlenia znaków jest w morslib.cpp
  mymors.queue('l', 0);
  mymors.queue('5');
  mymors.queue('a', 1);
  mymors.queue('a', 0);

  /*
  WAŻNE: 
  Jeżeli kolejkujemy znak z priorytetem 0, jest sprawdzane, czy ten znak pojawia się już gdzieś w kolejce. Jeżeli tak, to kolejkowanie nie następuje.
  (żeby nie zapchać bufora)
  Podobnie w przypadku kolejkowania z priorytetem 1. Tam podany znak porównywany jest z 3 pierwszymi miejscami bufora
  */
}


void loop()
{  
  mymors.handle();

  /*
  reszta kodu użytkownika
  */
}
