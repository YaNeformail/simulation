В рамках работы реализован узел, предназначенный для симмуляции робота с дифференциальным приводом, управление осуществляется с помощью пакета turtlebot3_teleop.

Требования к узлу:
1. Подписывается на управляющие команды для линейной и угловой скоростей в формате geometry_msgs/Twist

2. Публикует показания энкодеров в пользовательском формате сообщений, содержащем поле типа std_msgs/Header и два uint32 поля для левого и правого энкодеров.

3. Публикует положение робота в формате nav_msgs/Odom

"Подписка" на управляющие команды для скоростей описана в файле listen.py
Показания энкодеров прописаны в файле Num.msg
Обработка входных данных и последующая публикация положения робота описаны в файле version2.py

Формирование файлов listen.py и Num.msg производятся по стандартной процедуре, version2.py нуждается в более подробном описании.
Для более удобной работы с данными создается основной класс MyPublisher(), в котором настраивается публикатор, дающий информацию о данных одометрии, пути и угловой скорости каждого из колес, и подписчик. Также инициализируюся начальные значения положения робота, угловой скорости и угла колес и методы класса, используемые в процессе обработки данных. 
В качестве необходимых методов выступают метод интегрирования integration(), метод transitProcess()

Результаты работы симмуляции представлены ниже:
1. Сообщения энкодера

![image](https://github.com/YaNeformail/simulation/assets/79791800/06cdb05a-239e-4627-870a-9b875608fe5f)

2. Отрисовка ориентации

![image](https://github.com/YaNeformail/simulation/assets/79791800/562af8fb-e9f9-4a39-8bd7-e83af7f9414e)

3. Неудачная попытка отрисовать путь

![image](https://github.com/YaNeformail/simulation/assets/79791800/3c189897-e432-4c76-af9f-89a151034256)
