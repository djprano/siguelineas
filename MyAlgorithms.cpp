/*
 *  Copyright (C) 1997-2011 JDERobot Developers Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *  Authors : Maikel González <m.gonzalezbai@gmail.com>,
 *
 */

//ANCHO 640 ALTO 480

#include "API.h"
#include <sys/time.h>


//Constantes
const int FILA1 = 478;

const int GIRO_RECUPERACION = 8;

/*
 INTROROB API:
 ---------------------------------------------------------
 Métodos para obtener los datos de los sensores:

 this->getMotorV: Este método devuelve la velocidad lineal (mm./s.) del robot.
 EJEMPLO: float v = this->getMotorV();

 this->getMotorW: Este método devuelve la velocidad rotacional (deg./s.) del robot.
 EJEMPLO: float w = this->getMotorW();

 this->getLaserData: Este método devuelve una estructura con dos campos, por un lado el número de lasers del robot (180) y por otro un vector de 180 posiciones en cada una de las cuales se almacena la distancia obtenida por el láser, estando relacionada cada posición del robot con el ángulo del láser. Ejemplo: jderobot::LaserDataPtr laser = this->getLaserData() ||
 USO: printf("laser[45]: %d\n", laser->distanceData[45]); ||
 USO: printf("numLasers: %d\n", laser->numLaser);

 this->getDistancesLaser: Este método devolvería únicamente el vector mencionado anteriormente.
 EJEMPLO: jderobot::IntSeq VectorDistances this->getDistancesLaser();

 this->getNumLasers: Este método devuelve el otro campo mencionado, el número de grados del láser.
 EJEMPLO: int numLasers = this->getNumLasers();

 this->getEncodersData: Este método devuelve una estructura con tres campos: robotx, roboty y robottheta, siendo la posición "x", "y" y su orientación "theta" respectivamente.
 EJEMPLO: jderobot::EncodersDataPtr myPosition getEncodersData(); ||
 USO: printf("myPosition = [%f, %f]\n", myPosition->robotx, myPosition->roboty);


 ---------------------------------------------------------

 Métodos para manipular los actuadores:

 this->setMotorV(float V): Con este método definimos la velocidad lineal (mm./s.) del robot.
 EJEMPLO: this->setMotorV(40.)

 this->setMotorW(float W): Con este método definimos la velocidad rotacional (deg./s.) del robot.
 EJEMPLO: this->setMotorW(10.)



 ---------------------------------------------------------

 Métodos gráficos (mundo 3D)
 
 imageCameras2openCV(): Obtiene las imágenes captadas por las cámaras y las transforma al formato IplImage
 para un mejor manejo de éstas por medio de OpenCV. Las imágenes son alojadas en:
 * this->imageCameraLeft  
 * this->imageCameraRight                                                                                                            

 pintaSegmento: Traza una línea entre dos puntos dados a partir de un color dado.
 USO:
 CvPoint3D32f aa,bb;
 CvPoint3D32f color;

 bb.x=this->destino.x;
 bb.y=this->destino.y;
 bb.z=0.;

 aa.x=encodersData->robotx;
 aa.y=encodersData->roboty;
 aa.z=0;

 color.x = 1.; // Red
 color.y = 0.; // Green
 color.z = 0.; // Blue
 this->pintaSegmento (aa, bb, color);

 drawProjectionLines: Traza líneas desde el origen de coordenadas a un punto seleccionado (click izquierdo) en una de las cámaras del robot.
 USO: this->drawProjectionLines();


 drawSphere: Dibuja una esfera dado un punto y un color
 USO: this->drawSphere(bb, color);

 x_click_cameraleft: Almacena la coordenada x del punto donde se ha hecho click en la camara izquierda
 y_click_cameraleft: Almacena la coordenada y del punto donde se ha hecho click en la camara izquierda
 x_click_cameraright: Almacena la coordenada x del punto donde se ha hecho click en la camara derecha
 y_click_cameraright: Almacena la coordenada y del punto donde se ha hecho click en la camara derecha

 graficas2opticas: Transforma un punto (pointX,pointY) a su equivalente en el sistema de referencia usado por las camaras (progeo)
 USO: 
 * int pointX; -> Podemos usar el punto obtenido al hacer click en una camara de la GUI  x_click_cameraleft
 * int pointY; -> Podemos usar el punto obtenido al hacer click en una camara de la GUI  y_click_cameraleft
 * HPoint2D Point2DCam -> punto en el sistema de referencia de la camara
 * this->graficas2opticas(pointX,pointY,&Point2DCam);
 * De tal forma que: 
 *      + Point2DCam.x -> contiene la coordenada "x"
 *      + Point2DCam.y -> contiene la coordenada "y"
 * 
 opticas2graficas: Transforma un punto 2D (Point2DCam) en el sistema de refercia de las camaras a su equivalente en el sistema de referencia de las imagenes que se muestran en el interfaz grafico
 USO: 
 * int pointX; 
 * int pointY; 
 * HPoint2D Point2DCam -> punto en el sistema de referencia de la camara
 * this->opticas2graficas(&pointX,&pointY,Point2DCam);
 * De tal forma que: 
 *      + pointX -> contiene la coordenada "x"
 *      + pointY -> contiene la coordenada "y"
 ---------------------------------------------------------

 Otros:

 destino: Variable que almacena las coordenadas de la pocición seleccionada en el mundo 3D con el botón central del ratón.
 EJEMPLO: printf ("destPoint = [%f, %f]\n", this->destino.x, this->destino.y);

 absolutas2relativas: Método que calcula la posicion relativa respecto del robot de un punto absoluto. El robot se encuentra en robotx, roboty con orientacion robotheta respecto al sistema de referencia absoluto.

 relativas2absolutas: Método que calcula la posicion absoluta de un punto expresado en el sistema de coordenadas solidario al robot. El robot se encuentra en robotx, roboty con orientacion robotheta respecto al sistema de referencia absoluto.
 USO:
 CvPoint3D32f aa,a,b;

 aa.x=0.; aa.y=0.;
 this->relativas2absolutas(aa,&a);
 aa.x = 1000.; aa.y = -2000.;  // en mm.
 this->relativas2absolutas(aa,&b);

 **** PARA MAS INFO ACCEDER AL FICHERO API.CPP y API.H ****

 */
int FILA3 = 320;
int L_FILTRO = 320;
int FILA2 = 382;

int estado = 4;
int gradosagirar = 0;
int gradosallegar = 0;
int iteraciones = 0;
int iteraciones_sec = 0;
struct timeval start;
struct timeval end;
struct timeval t;
struct timeval t2;
int lastError[3];
int n_fren;
int estado2_estable;

namespace introrob {

void Api::RunNavigationAlgorithm() {
	double v, w, l, pan, tilt;
	jderobot::LaserDataPtr laser;
	CvPoint2D32f dest;
	int i, j;
	jderobot::EncodersDataPtr encoders;
	if(t.tv_sec==0){
		gettimeofday(&t, NULL);
	}

	/*A PARTIR DE AQUÍ SE PUEDE AÑADIR EL CÓDIGO DE NAVEGACIÓN IMPLEMENTADO POR EL ESTUDIANTE*/

	/*ALGUNOS EJEMPLOS*/

	/*Manipulando imágenes de las cámaras*/
	/*En el siguiente ejemplo se filtra el color rojo de la cámara izquierda para repintar esos píxeles a negro. Para visualizar el resultado
	 debemos desplegar la ventana "WINDOW DEBUGGING" y pulsar PLAY para hacer correr nuestro código*/
	imageCameras2openCV(); //Esta función es necesario llamarla ANTES de trabajar con las imágenes de las cámaras.
	IplImage src = *this->imageCameraLeft; //Imagen de la cámara izquierda
	
	//Invertimos el Rojo por el azul para que los colores se dibujen bien en la ventana de depuración.
//		for (i = 0; i < src.width; i++) {
//			for (j = 0; j < src.height; j++) {
//			int R = (int) (unsigned char)src.imageData[(j * src.width + i) * src.nChannels + 2];
//			src.imageData[(j * src.width + i) * src.nChannels + 2] =
//					src.imageData[(j * src.width + i) * src.nChannels];
//			src.imageData[(j * src.width + i) * src.nChannels] = R;
//		}
//	}
	
	int acumulador1 = 0;
	int acumulador2 = 0;
	int acumulador3 = 0;
	int valorMedio1 = 0;
	int valorMedio2 = 0;
	int valorMedio3 = 0;
	int error1 = 0;
	int error2 = 0;
	int error3 = 0;

	//filtro de linea y pinto lineas de referencia
	for (j = L_FILTRO ; j < src.height; j++) {
		for (i = 0; i < src.width; i++) {
			if (((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels] > 45)
					&& ((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels + 1] < 25)
					&& ((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels + 2] < 25)) {
				src.imageData[(j * src.width + i) * src.nChannels] = 0; //R
				src.imageData[(j * src.width + i) * src.nChannels + 1] = 254; //G
				src.imageData[(j * src.width + i) * src.nChannels + 2] = 254; //B
			}
			if (j == FILA1) {
				
				if (((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels] == 0)
						&& ((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels + 1] == 254)
						&& ((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels + 2] == 254)) {
					acumulador1++;
					valorMedio1 = valorMedio1 + i;
					src.imageData[(j * src.width + i) * src.nChannels] = 0; //R
					src.imageData[(j * src.width + i) * src.nChannels + 1] = 254; //G
					src.imageData[(j * src.width + i) * src.nChannels + 2] = 0; //B
				}

			}
			if (j == FILA2) {
				if (((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels] == 0)
						&& ((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels + 1] == 254)
						&& ((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels + 2] == 254)) {
					acumulador2++;
					valorMedio2 = valorMedio2 + i;
					src.imageData[(j * src.width + i) * src.nChannels] = 0; //R
					src.imageData[(j * src.width + i) * src.nChannels + 1] = 254; //G
					src.imageData[(j * src.width + i) * src.nChannels + 2] = 0; //B
				}

			
			}
			if (j == FILA3) {
				if (((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels] == 0)
						&& ((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels + 1] == 254)
						&& ((int) (unsigned char) src.imageData[(j * src.width + i)* src.nChannels + 2] == 254)) {
					acumulador3++;
					valorMedio3 = valorMedio3 + i;
					src.imageData[(j * src.width + i) * src.nChannels] = 0; //R
					src.imageData[(j * src.width + i) * src.nChannels + 1] = 254; //G
					src.imageData[(j * src.width + i) * src.nChannels + 2] = 0; //B
				}
				
			}
		}
	}
	
		//pinto los valores medios
		if(acumulador1!=0){
			valorMedio1=valorMedio1/acumulador1;
			src.imageData[(FILA1*src.width+valorMedio1)*src.nChannels]=0;
			src.imageData[(FILA1*src.width+valorMedio1)*src.nChannels+1]=0;
			src.imageData[(FILA1*src.width+valorMedio1)*src.nChannels+2]=0;
		}
		if (acumulador2!=0){
			valorMedio2=valorMedio2/acumulador2;
			src.imageData[(FILA2*src.width+valorMedio2)*src.nChannels]=0;
			src.imageData[(FILA2*src.width+valorMedio2)*src.nChannels+1]=0;
			src.imageData[(FILA2*src.width+valorMedio2)*src.nChannels+2]=0;

		}
		if (acumulador3!=0){
			valorMedio3=valorMedio3/acumulador3;
			src.imageData[(FILA3*src.width+valorMedio3)*src.nChannels]=0;
			src.imageData[(FILA3*src.width+valorMedio3)*src.nChannels+1]=0;
			src.imageData[(FILA3*src.width+valorMedio3)*src.nChannels+2]=0;
		}

		
	//calculamos el error para la toma de decisión
		
		error1 = (src.width/2)-valorMedio1;
		error2 = (src.width/2)-valorMedio2;
		error3 = (src.width/2)-valorMedio3;
	
	//MAQUINA DE ESTADOS
	switch (estado){
		
		//Velocidad maxima con correccion de volante suave
		case 0:
			L_FILTRO = 295;
			FILA3 = L_FILTRO;
			FILA2 =350;
			this->setMotorV(130);
			this->setMotorW(error2*0.007);
			if(acumulador1==0 && acumulador2==0 && acumulador3==0){
				estado=4;
			}else if(abs(error3)>100||error2>100){
				estado=1;
				n_fren=0;
			}
			break;
		//1 frenar con correcion de volante fuerte
		case 1:
			FILA2=400;
			L_FILTRO = 370;
			FILA3=L_FILTRO;
			this->setMotorV(1);
			if(acumulador3!=0){
				this->setMotorW(error3*1.5);
			}else if(acumulador2!=0){
				this->setMotorW(error2*1.5);
			}

			if(n_fren>3){
				if (acumulador1!=0 && acumulador2!=0&&acumulador3!=0){
					estado=4;
				}else{
					estado=3;
				}
				
			}else{
				n_fren++;
			}
			break;
		//2 curva rapida velocidad media con corrección de volante más dura
		case 2:
			FILA2=392;
			L_FILTRO = 325;
			FILA3=L_FILTRO;
			this->setMotorV(65);
			if(acumulador2!=0){
				this->setMotorW(error2*0.02);
			}
			
			if(acumulador1==0 && acumulador2==0 && acumulador3==0){
				estado=4;
				estado2_estable=0;
			}else if(abs(error3)>150){
				estado=3;
				estado2_estable=0;
			}else if (abs(error2)<30 && abs(error3)<55 && abs(error1)<30){
				if(estado2_estable>5){
					estado=0;
					estado2_estable=0;
				}else{
					estado2_estable++;
				}
				
			}else{
				estado2_estable=0;
			}
			break;
		//3 curva lenta  velocidad baja
		case 3:
			FILA2=392;
			L_FILTRO = 360;
			FILA3=L_FILTRO;
			this->setMotorV(48);
			if(acumulador3!=0){
				this->setMotorW(error3*0.045);
			}else if(acumulador2!=0){
				this->setMotorW(error2*0.036);
			}else if(acumulador1!=0){
				this->setMotorW(error1*0.029);
			}else{
				estado=4;
			}
			
			if(abs(error2)<80 && abs(error3)<180){
				estado=2;
			}
			break;
		//4 recuperar, se salió del circuito
		case 4:
			L_FILTRO = 340;
			FILA3 = L_FILTRO;
			this->setMotorV(2);
			if(lastError[1]!=0){
				if(lastError[1]>0){
					this->setMotorW(GIRO_RECUPERACION);
				}else{
					this->setMotorW(-GIRO_RECUPERACION);
				}
				
			}
			if (acumulador3!=0){
				estado=5;
			}
		break;
		//5 andar hasta el circuito
		case 5:
			this->setMotorV(15);

			this->setMotorW(error3*0.024);
			
			if (acumulador2!=0){
				estado=3;
			}
		break;

	}
	if(acumulador1!=0){
		lastError[0]=error1;
	}
	if(acumulador2!=0){
		lastError[1]=error2;
	}
	if(acumulador3!=0){
		lastError[2]=error3;
	}

	
//calculo iteraciones por segundo	
	gettimeofday(&t2, NULL);
	if(t2.tv_sec-t.tv_sec>5){
		gettimeofday(&t, NULL);
		iteraciones_sec = iteraciones/5;
		iteraciones =0;
	}else{
		iteraciones++;
	}
	

	

	/* A continuacion se muestran las coordenadas de los puntos obtenidos tras hacer click en alguna de las camaras */
//std::cout << x_click_cameraleft << std::endl; // Coordenada x del punto donde se ha hecho click en la camara izquierda
//std::cout << y_click_cameraleft << std::endl; // Coordenada y del punto donde se ha hecho click en la camara izquierda
//std::cout << x_click_cameraright << std::endl; // Coordenada x del punto donde se ha hecho click en la camara derecha
//std::cout << y_click_cameraright << std::endl; // Coordenada y del punto donde se ha hecho click en la camara derecha
	/* EJEMPLO SENCILLO DE UN BUMP AND GO */

	/* TOMA DE SENSORES */
//Aqui tomamos el valor de los sensores para alojarlo en nuestras variables locales
	laser = getLaserData(); // Get the laser info
	encoders = this->getEncodersData();
	// printf("%d -- %d -- %d Estado: %i iteraciones/sec: %i rgb: %i-%i-%i velocidad: %i \n",
	// 		laser->distanceData[174], laser->distanceData[90],
	// 		laser->distanceData[5],estado, iteraciones_sec,(int) (unsigned char) src.imageData[((src.height-1) * src.width + (src.width/2))* src.nChannels],(int) (unsigned char) src.imageData[((src.height-1) * src.width + (src.width/2))* src.nChannels+1],
	// 		(int) (unsigned char) src.imageData[((src.height-1) * src.width + (src.width/2))* src.nChannels+2],velocidad);
	printf("Estado: %i iteraciones/sec: %i rgb: %i-%i-%i ",estado, iteraciones_sec,(int) (unsigned char) src.imageData[((src.height-1) * src.width + (src.width/2))* src.nChannels],(int) (unsigned char) src.imageData[((src.height-1) * src.width + (src.width/2))* src.nChannels+1],
			(int) (unsigned char) src.imageData[((src.height-1) * src.width + (src.width/2))* src.nChannels+2]);
	printf("Error1: %i Error2: %i Error3: %i lastError:%i MotorW:%g \n",error1,error2,error3,lastError[2],-getMotorW());

	v = this->getMotorV();
	w = this->getMotorW();
	l = this->getMotorL();
//        printf("v: %f , w: %f , l: %f\n", v, w, l);

//dest = this->getDestino();
//printf("destPoint = [%f, %f]\n", dest.x, dest.y);

//printf("myPosition = [%f, %f]\n", encoders->robotx, encoders->roboty);


	/*
	 switch(accion){

	 case 0:		// Robot hacia adelante

	 if(( laser->distanceData[45] < 1000.0) or ( laser->distanceData[90] < 1000.0) or ( laser->distanceData[135] < 1000.0)){
	 v=0.;
	 //if ((x_ant == myPoint.x) and (y_ant == myPoint.y) and (z_ant == myPoint.z)){
	 accion=1;
	 printf("### Activado hacia Atras\n");
	 //}
	 }
	 else
	 v=50;
	 break;

	 case 1:		// Robot hacia atras
	 if ((laser->distanceData[45] < 1100) or (laser->distanceData[90] < 1100) or (laser->distanceData[135] < 1100)){
	 v=-50.;
	 printf("### Llendo hacia atras\n");
	 }
	 else{
	 v=0.;
	 accion=2;
	 }
	 break;


	 case 2:		// Robot girando.
	 if((laser->distanceData[45] < 1300) or (laser->distanceData[90] < 1300) or (laser->distanceData[135] < 1300)){
	 if(sentido%2==0){
	 w=50.;
	 }
	 else{
	 w=50.*(-1);
	 }
	 printf("### Girando: %d \n", sentido);
	 }
	 else{
	 w=0.;
	 accion=0;
	 sentido = (1 + rand() % 40);
	 }

	 break;
	 }
	 */
	/*Comandar robot*/
//Aqui enviamos los datos al robot
//w = 0.;
//        this->setMotorW(0);
//        this->setMotorV(9);
//
//        this->setMotorL(0);
//pan=0;
//tilt=0;
//this->setPTEncoders(pan,tilt,1); //Con el segundo parametro elegimos la camara a comandar (1 o 2)
}

void Api::RunGraphicsAlgorithm() {
	/* TODO: ADD YOUR GRAPHIC CODE HERE */
	CvPoint3D32f aa, bb;
	CvPoint3D32f a, b;
	CvPoint3D32f c, d;
	CvPoint3D32f color;

//        // Init camera 1
//        camera *mycameraA = new camera("cameras/calibA");
//        myCamA = mycameraA->readConfig();
	xmlReader(&myCamA, "cameras/calibA.xml");
//
//        // Init camera 2
//        camera *mycameraB = new camera("cameras/calibB");
//        myCamB = mycameraB->readConfig();
	xmlReader(&myCamB, "cameras/calibB.xml");
//        ///////////// EJEMPLO DE USO DE OPTICAS2GRAFICAS y GRAFICAS2OPTICAS ////////////////////
//        HPoint2D punto2Daux; //punto 2D (graficas)
//        HPoint3D punto3Daux; //punto 3D (opticas)
//        double x;
//        double y;
//        
//        
//        this->graficas2opticas(punto2D1.x,punto2D1.y,&punto2Daux); // 
//        
//        backproject(&punto3D1, punto2Daux, myCamA);
//
//
//	click1.x=punto3D1.X; 
//	click1.y=punto3D1.Y;
//	click1.z=punto3D1.Z;  
//
//        this->opticas2graficas(punto2Daux, &x, &y);
//        std::cout << "Xafter: " <<  x << "Yafter: " << y << std::endl;
//        
//        ///////////// FIN EJEMPLO DE USO DE OPTICAS2GRAFICAS y GRAFICAS2OPTICAS ////////////////////

	bb.x = this->destino.x;
	bb.y = this->destino.y;
	bb.z = 0.;

	aa.x = encodersData->robotx;
	aa.y = encodersData->roboty;
	aa.z = 0;

	color.x = 1.; // Red
	color.y = 0.; // Green
	color.z = 0.; // Blue
	this->pintaSegmento(aa, bb, color); // ROJO - Pinta un segmento desde el punto "aa" hasta el punto "bb"
	this->pintaDestino(aa, bb, color); // ROJO - Marca con una estrella el destino seleccionado al hacer click con el botón central en el mundo 3D.
	this->drawSphere(bb, color);

	/* this->drawProjectionLines();*/

}

}
