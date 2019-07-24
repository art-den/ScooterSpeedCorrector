#  Делаем старт электросамоката менее дёрганным

Небольшое электронное устройство на 8-ногом микроконтроллере ATTiny позволит сделать старт самоката менее дёрганным, а управление на низкой скорости более комфортным.

Устройство подходит только для электросамокатов, у которых ручка (или курок) газа подключается отдельно к контролеру! Если у вас курок газа располагается на дисплее контроллера, то использовать данное устройство не получится!

Устройство подключается в разрыв между ручкой газа и BLDC-контроллером электросамоката.

## Как работает это устройство?


## Что нужно для изготовления данного устройства?
1. Микроконтроллер ATTiny серий attiny25, attiny45, attiny85. Лучше всего использовать ATTiny85, т. к. скорее всего прошивка будет совершенствоваться и в будущем может не полезть в младшие серии  attiny25, attiny45.
1. Программатор для микроконтроллеров AVR. Не обязательно брать дорогой программатор. Подойдёт любой дешевый клон.
1. Два электролитических конденсатора 100 мкФ на 6.3 вольта и более
1. Один мелкий керамический конденсатор 100 нФ
1. Два резистора 300 Ом
1. Один резистор 10 кОм
1. Разъёмы для подключения к микроконтроллеру к ручке газа
1. Разъём для прошивки

## Схема устройства
Схема устройства выглядит следующим образом:

![Схема](/images/SpeedCorr.GIF)

Аналоговый сигнал поступает с ручки газа на 2-ю ногу микроконтроллера и измеряется при помощи АЦП. Т.к. у данной серии микроконтроллеров нету ЦАП для выдачи аналогового сигнала, то выходной сигнал получается при помощи ШИМ (6-я нога микроконтроллера) и сглаживается RC-цепочкой R2,C3.

## Сборка устройства
Устройство собирается без платы навесным монтажом (элементы напаиваются прямо на микросхему микроконтроллера). Можно всё это собрать и на плате, но в этом нету сильной необходимости, т. к. число составных элементов небольшое.

## Прошивка устройства
Перед тем, как залить прошивку в устройство, её надо скомпилировать. Для компиляции прошивки необходим компилятор AVR-GCC. Его можно скачать [по этой ссылке](http://blog.zakkemble.net/avr-gcc-builds/). После того, как вы распаковали компилятор в папку на своём компьютере, надо дописать в переменную PATH путь к папке bin компилятора (например, D:\avr-gcc-9.1.0-x86-mingw\bin).

Для программатора прошивки могут потребоваться драйверы. Найти драйверы можно по поисковой фразе <название программатора> drivers.

Откуда брать прошивку? Прошивка скачивается с этой страницы при помощи зелёной кнопки «Clone or download». Нажмите на эту кнопку, а затем на «Download ZIP». После этого на ваш компьютер запишется архив с прошивкой. Ещё надо распаковать в удобную для вас папку.

Компиляция и прошивка. Зайдите в папку firmware распакованной прошивки. Запустите файл build-and-flash.bat. Этот файл скомпилирует прошивку и, если программатор подключен к компьютеру, а устройство к программатору, то сразу прошьёт устройство.