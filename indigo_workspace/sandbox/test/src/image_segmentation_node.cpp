




//Este programa lee una imagen RGB en formato BMP, obtiene la componente de
//intensidad correspondiente, asi como dubuja un segmento lineal y un circulo
//sobre ella. Los resultados se almacenan en archivos en formato YUV400 y BMP.
//Las dimensiones de las imagenes, el directorio de entrada, el directorio de
//salida, y los parametros para dibujar el segmento lineal y el circulo se
//obtienen de un archivo de parametros de control.

//***************COMIENZO PASO (1)***********
//*******************************************
//*********De codeBlocks a ROS***************
//*******************************************

//Se incluyen los siguientes headers para poder hacer uso
//de funciones de ROS en nuestro programa
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//Se incluyo el siguiente header para poder hacer uso de
//funciones de OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Se incluyen los siguientes prototipos de funciones
void geoDisplayInWindowIntensityImage(const unsigned char *pIntensityImage, char windowName[256]);
void geoCopyTheRGBImageDataFromTheOpencvImageStructureToOurpInputImageStructure();
void geoFunctionToHandlePublishedImage(const sensor_msgs::ImageConstPtr& msg);

//Se definen las siguientes variables globales
image_transport::Subscriber sub_ourImageTopic_;
cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr cv_ptr_display;
namespace enc = sensor_msgs::image_encodings;
int contadorDeImagenesRecibidas=0;
char windowName0[256];
char windowName1[256];
char windowName2[256];
char windowName3[256];
char windowName4[256];
char windowName5[256];
char windowName6[256];
char windowName7[256];
//*******************************************
//*********De codeBlocks a ROS***************
//*******************************************
//***************FIN PASO (1)****************




#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>


void geoLeerParametrosDeControlDeArchivoDeTexto();
void readRGBImageFromBMPFile(char *filename);
void geoInsertYourCodeHere();
void geoGetIntensityImageFromRGBimage();
void getHistogramAndProbabilityDensityFunctionOfIntensityValues();
void getMeanMeanOfSquaresAndVarianceOfIntensityValues();
void umbralizacionkittler();
void Save_umbralizacion_results_in_text_file();
int SaveIntensityImageIn_BMP_file(unsigned char *pintensity, char *filename);
int SaveRGBImageIn_BMP_file(unsigned char *prgb, char *filename);

void beSaveResultsOfProbabilityInTextFile();
void geoDrawALinealSegmentOnIntensityImage();
void geoDrawACircleOnIntensityImage();
void geoSaveIntensityImageIn_YUV400_file(unsigned char *pIntensity, char* filename);
void geoChangeImageCoordinateSystemFromLowerLeftSideToUpperLeftSide(unsigned char *pIntensity, unsigned char *presult);
void beDrawATriangleOnIntensityImage();

//Funciones para probabilidad e histogramas
void geoPrintOnTerminalAllRgbValuesOfTheRgbImage();
void geoSegmentIntesityImageByManualThresholding();

void geoGetHistogramAndProbabilityDensityFunctionOfIntensityValues();
void geoGetMeanMeanOfSquaresAndVarianceOfIntensityValues();
void geoGetMeanImage();
void geoGetVarianceImage();

//Contenedor de imagenes

struct pInputImage
{
    int width;  //ancho de imagenes
    int height; //alto de imagenes
    unsigned char *prgb; //imagen rgb de entrada
    unsigned char *pintensity; //imagen de intensidad
    int histograma[256];
    double p[256];
    double m, ms, var;
    unsigned char *pdrawnLinealSegmentOnIntensity; //imagen resultado
    unsigned char *pdrawnCircleOnIntensity; //imagen resultado
    unsigned char *pdrawnTriangleOnIntensity; //imagen resultado
    double medidas_estadisticas[3];
    int funcion_vero;
    double c1, c2, m1, m2, varianza1, varianza2;
    unsigned char *pimagensegmentada;
    double *pmeanImage;
    unsigned char *pmeanImage_uc;
    double *pvarianceImage;
    unsigned char *pvarianceImage_uc;
};

//Contenedor de parametros de control
struct contenedor_de_parametros_de_control
{
    int width; //ancho de las imagenes
    int height; //alto de las imagenes
    char pathAndInputImageFileName[256]; //directorio de entrada
    char pathOfOutputDirectory[256]; //directorio de salida
    int xi; //(xi,yi) punto inicial del segmento lineal
    int yi;
    int xf; //(xf,yf) punto final del segmento lineal
    int yf;
    int cx; //(cx,cy) centro del circulo
    int cy;
    int r; //radio del circulo
    int v1x; //vertice x 1
    int v1y; //vertice y 1
    int v2x;//vertice x 2
    int v2y; //vertice y 2
    int v3x;//vertice x 3
    int v3y; //vertice y 3
};


//***
//***Insertar aqui las definiciones de variables globales,
//***que son aquellas variables que se podran acceder desde
//***cualquier funcion dentro de este archivo
//***

//El siguiente puntero global apuntara al contenedor que
//usaremos para guardar los valores de control de flujo
//del programa que se leeran de un archivo de texto

struct contenedor_de_parametros_de_control *p_parametros;

//El siguiente puntero global apuntara al contenedor que
//usaremos para guardar las imagenes que utilizaremos
//en el programa

struct pInputImage *pInputImage;

//La siguiente variable global se usara como contador
//el numero de datos leidos

int numeroDeDatosLeidos=0;

//***
//***Insertar aqui las constantes del programa
//***

#define PI 3.141592652

//Inicio de programa principal

//Inicio de programa principal

//***************COMIENZO PASO (2)***********
//*******************************************
//*********De codeBlocks a ROS***************
//*******************************************

//CAMBIAR:

//int main()

//POR:

int main(int argc, char** argv)

//*******************************************
//*********De codeBlocks a ROS***************
//*******************************************
//***************FIN PASO (2)****************

{
    //definición de variables locales
    int i; //contador
    int width, height;
    //Despliegue de autoría en el terminal

    printf("****************************************************************************\n");
    printf("** II EXAMEN DE DISEÑO PROGRAMACIÓN Y PRUEBA, Belinda Brown B61254         **\n");
    printf("** Image Segmentation Project   ROS NODE                                   **\n");
    printf("** Programa de referencia tomado del Prof. Dr.-Ing. Geovanni Martínez      **\n");
    printf("** IE-0449 Vision por Computador                                           **\n");
    printf("** I-2019                                                                  **\n");
    printf("****************************************************************************\n");
    printf("\n");

    printf("Recuerde que si desea utilizar otra imagen que poseee diferentes dimensiones, debe cambiar\nlas dimensiones y el nombre en el archivo de parámetros de control el cual es el de current_control_parameters.txt \n");


    //Reservando memoria de contenedor p_parametros
    p_parametros = (struct contenedor_de_parametros_de_control *)malloc(sizeof(struct contenedor_de_parametros_de_control));

    //Esta función lee los parámetros de control de flujo del
    //programa desde un archivo de texto y los almacena en el
    //contenedor p_parametros
    geoLeerParametrosDeControlDeArchivoDeTexto();

    //Reservando memoria para la estructura pInputImage
    pInputImage = (struct pInputImage *)malloc(sizeof(struct pInputImage));
    pInputImage->width=p_parametros->width;
    pInputImage->height=p_parametros->height;

    //Reservando e inicializando la memoria de las imagenes del contenedor pInputImage
    width=p_parametros->width;
    height=p_parametros->height;
    pInputImage->prgb = (unsigned char*)malloc(sizeof(unsigned char)*width*height*3);
    pInputImage->pintensity =(unsigned char*)malloc(sizeof(unsigned char)*width*height);
    pInputImage->pimagensegmentada=(unsigned char*)malloc(sizeof(unsigned char)*width*height);
    pInputImage->pmeanImage =(double *)malloc(sizeof(double)*width*height);
    pInputImage->pmeanImage_uc =(unsigned char*)malloc(sizeof(unsigned char)*width*height);
    pInputImage->pvarianceImage= (double *)malloc(sizeof(double)*width*height);
    pInputImage->pvarianceImage_uc= (unsigned char*)malloc(sizeof(unsigned char)*width*height);
    pInputImage->pdrawnLinealSegmentOnIntensity =(unsigned char*)malloc(sizeof(unsigned char)*width*height);
    pInputImage->pdrawnCircleOnIntensity =(unsigned char*)malloc(sizeof(unsigned char)*width*height);
    pInputImage->pdrawnTriangleOnIntensity =(unsigned char*)malloc(sizeof(unsigned char)*width*height);

    //Cada píxel se inicializa en cero

    for (i=0;i<width*height*3;i++) pInputImage->prgb[i]=0;
    for (i=0;i<width*height;i++) {
        pInputImage->pintensity[i]=0;
        pInputImage->pimagensegmentada[i]=0;
        pInputImage->pdrawnLinealSegmentOnIntensity[i]=0;
        pInputImage->pdrawnCircleOnIntensity[i]=0;
        pInputImage->pdrawnTriangleOnIntensity[i]=0;
        pInputImage->pmeanImage[i]=0.0;
        pInputImage->pmeanImage_uc[i]=0;
        pInputImage->pvarianceImage[i]=0.0;
        pInputImage->pvarianceImage_uc[i]=0;
        }

        //Cada espacio del arreglo para el histograma y la
        //densidad de probabilidad se inicializa en cero
    for(i=0; i<256; i++)
    {
        pInputImage->histograma[i]=0;
        pInputImage->p[i]=0.0;
    }
    //***************COMIENZO PASO (3)***********
    //*******************************************
    //*********De codeBlocks a ROS***************
    //*******************************************
    //CAMBIAR:
    //Leyendo la imagen RGB de archivo en formato BMP
    //readRGBImageFromBMPFile(p_parametros->pathAndInputImageFileName);

    //Insertar codigo en esta funcion
  //  geoInsertYourCodeHere();

  //POR:

  //Levantando del nodo ROS llamado "image_segmentation_node".
  ros::init(argc, argv, "image_segmentation_node");
  ros::NodeHandle nh_;

  //Dandole al nodo la capacidad de recepcion de
  //mensajes con imagenes
  image_transport::ImageTransport it_(nh_);

  //Subscripción al topico "/usb_cam/image_raw", a traves del cual
  //se recibiran las imagenes capturadas por la camara usb de su
  //laptop. Se define un buffer de entrada de máximo 1 imágenes
  sub_ourImageTopic_ = it_.subscribe("/usb_cam/image_raw", 1,
                     geoFunctionToHandlePublishedImage);
  //La función geoFunctionToHandlePublishedImage se ejecutara cada vez
  //que un mensaje se reciba a través del tópico "/usb_cam/image_raw".
  //Creando ventanas OpenCV para desplegar imagenes
  strcpy(windowName0,"Imagen RGB recibida en mensaje");
  strcpy(windowName1,"Imagen de Intensidad");
  strcpy(windowName2,"Segmento lineal");
  strcpy(windowName3,"Circulo");
  strcpy(windowName4,"Imagen Segmentada");
  strcpy(windowName5,"meanImage");
  strcpy(windowName6,"triangle");
  strcpy(windowName7,"varianceImage");
  cv::namedWindow(windowName0);
  cv::moveWindow(windowName0, 0, 0);
  cv::namedWindow(windowName1);
  cv::moveWindow(windowName1, 300, 0);
  cv::namedWindow(windowName2);
  cv::moveWindow(windowName2, 0, 200);
  cv::namedWindow(windowName3);
  cv::moveWindow(windowName3, 300, 200);
  cv::namedWindow(windowName4);
  cv::moveWindow(windowName4, 0, 100);
  cv::namedWindow(windowName5);
  cv::moveWindow(windowName5, 0, 300);
  cv::namedWindow(windowName6);
  cv::moveWindow(windowName6, 200,300);
  cv::namedWindow(windowName7);
  cv::moveWindow(windowName7, 100,0);
  cvWaitKey(30); //Esta funcion ademas de hacer esperar al
  //programa 30 ms, tambien fuerza a OpenCv a crear
  //inmediatamente las ventanas

  //El valor de X en la función ros::Rate loop_rate(X)
  //indica el número de ciclos "while (ros::ok())" que ROS
  //deberá realizar por segundo aproximadamente. Esta función
  //trabaja en forma conjunta con la función loop_rate.sleep()
  ros::Rate loop_rate(20);

  //ros::ok() es cero cuando ctrl+c es presionado en el teclado.
  //Utilice esa combinación de teclas para salirse del programa.
  while (ros::ok()) {

       //Dentro de la funcion "ros::spinOnce()" ROS ejecuta
       //sus funciones. Los mensajes se atenderán solamente
       //dentro de "ros::spinOnce()".
       ros::spinOnce();

       loop_rate.sleep();
  }

  //*******************************************
  //*********De codeBlocks a ROS***************
  //*******************************************
  //***************FIN PASO (3)****************


    //Liberando memoria de los contenedores e imagenes
    free(pInputImage->prgb);
    free(pInputImage->pintensity);
    free(pInputImage->pdrawnLinealSegmentOnIntensity);
    free(pInputImage->pdrawnCircleOnIntensity);
    free(pInputImage->pdrawnTriangleOnIntensity);
    free(pInputImage->pimagensegmentada);
    free(pInputImage->pmeanImage);
    free(pInputImage->pmeanImage_uc);
    free(pInputImage->pvarianceImage);
    free(pInputImage->pvarianceImage_uc);
    free(pInputImage);
    free(p_parametros);

    return 0;
}

//Fin de programa principal

//*******************************************************
//*******************************************************
//***** Introduzca aqui sus funciones               *****
//*******************************************************
//*******************************************************

//Esta funcion es para insertar nuevo codigo.
void geoInsertYourCodeHere()
{
    char pathAndFileName[256];
    strcpy(pathAndFileName,"output/rgb.bmp");
    SaveRGBImageIn_BMP_file(pInputImage->prgb, pathAndFileName);

    //Calculando la imagen de intensidad
    geoGetIntensityImageFromRGBimage();
    //Almacenando resultado en archivo en formato YUV400
    strcpy(pathAndFileName,"output/imagenDeIntensidad.yuv");
    geoSaveIntensityImageIn_YUV400_file(pInputImage->pintensity, pathAndFileName);
    strcpy(pathAndFileName,"output/imagenDeIntensidad.bmp");
    SaveIntensityImageIn_BMP_file(pInputImage->pintensity, pathAndFileName);

    //Dibujando segmento lineal sobre imagen de intensidad
    geoDrawALinealSegmentOnIntensityImage();
    //Almacenando resultado en archivo en formato YUV400
    strcpy(pathAndFileName,"output/linearSegment.yuv");
    geoSaveIntensityImageIn_YUV400_file(pInputImage->pdrawnLinealSegmentOnIntensity, pathAndFileName);
    strcpy(pathAndFileName,"output/linearSegment.bmp");
    SaveIntensityImageIn_BMP_file(pInputImage->pdrawnLinealSegmentOnIntensity, pathAndFileName);

    //Dibujando circulo sobre imagen de intensidad
    geoDrawACircleOnIntensityImage();
    //Almacenando resultado en archivo en formato YUV400
    strcpy(pathAndFileName,"output/circle.yuv");
    geoSaveIntensityImageIn_YUV400_file(pInputImage->pdrawnCircleOnIntensity, pathAndFileName);
    strcpy(pathAndFileName,"output/circle.bmp");
    SaveIntensityImageIn_BMP_file(pInputImage->pdrawnCircleOnIntensity, pathAndFileName);

    //Dibujando triángulo sobre imagen de intensidad
    beDrawATriangleOnIntensityImage();
    //Almacenando resultado en archivo en formato YUV400
    strcpy(pathAndFileName,"output/triangle.yuv");
    geoSaveIntensityImageIn_YUV400_file(pInputImage->pdrawnTriangleOnIntensity, pathAndFileName);
    strcpy(pathAndFileName,"output/triangle.bmp");
    SaveIntensityImageIn_BMP_file(pInputImage->pdrawnTriangleOnIntensity, pathAndFileName);
    //Imagen de medias
    geoGetMeanImage();

    //Almacenando resultado de medias en archivo en formato YUV400
    strcpy(pathAndFileName,"output/meanImage.yuv");
    geoSaveIntensityImageIn_YUV400_file(pInputImage->pmeanImage_uc, pathAndFileName);
    strcpy(pathAndFileName,"output/meanImage.bmp");
    SaveIntensityImageIn_BMP_file(pInputImage->pmeanImage_uc, pathAndFileName);

    //Imagen de varianzas
    geoGetVarianceImage();

    //Almacenando resultado en archivo en formato YUV400
    strcpy(pathAndFileName,"output/varianceImage.yuv");
    geoSaveIntensityImageIn_YUV400_file(pInputImage->pvarianceImage_uc, pathAndFileName);
    strcpy(pathAndFileName,"output/varianceImage.bmp");
    SaveIntensityImageIn_BMP_file(pInputImage->pvarianceImage_uc, pathAndFileName);






    getHistogramAndProbabilityDensityFunctionOfIntensityValues();

    getMeanMeanOfSquaresAndVarianceOfIntensityValues();

    //  umbralizacionkittler

    umbralizacionkittler();
    //Almacenando resultado en archivo en formato YUV400
    strcpy(pathAndFileName,"output/imagenSegmentada.yuv");
    geoSaveIntensityImageIn_YUV400_file(pInputImage->pimagensegmentada, pathAndFileName);
    strcpy(pathAndFileName,"output/imagenSegmentada.bmp");
    SaveIntensityImageIn_BMP_file(pInputImage->pimagensegmentada, pathAndFileName);

    //Almacena resultados en un .txt

    beSaveResultsOfProbabilityInTextFile();

    Save_umbralizacion_results_in_text_file();



    //***************COMIENZO PASO (4)***********
    //*******************************************
    //*********De codeBlocks a ROS***************
    //*******************************************

    //AGREGAR:

    //Desplegando imagenes en las ventanas OpenCV
    geoDisplayInWindowIntensityImage(pInputImage->pintensity, windowName1);
    geoDisplayInWindowIntensityImage(pInputImage->pdrawnLinealSegmentOnIntensity, windowName2);
    geoDisplayInWindowIntensityImage(pInputImage->pdrawnCircleOnIntensity, windowName3);
    geoDisplayInWindowIntensityImage(pInputImage->pimagensegmentada, windowName4);
    geoDisplayInWindowIntensityImage(pInputImage->pmeanImage_uc, windowName5);
    geoDisplayInWindowIntensityImage(pInputImage->pdrawnTriangleOnIntensity, windowName6);
    geoDisplayInWindowIntensityImage(pInputImage->pvarianceImage_uc, windowName7);


    cvWaitKey(30); //para que se desplieguen de una vez

    //*******************************************
    //*********De codeBlocks a ROS***************
    //*******************************************
    //***************FIN PASO (4)****************

}

    //Esta funcion obtiene la imagen de intensidad de
    //una imagen RGB
void geoGetIntensityImageFromRGBimage()
{
    int i,j;
    int width, height;

    //Renombrando para facilitar código
    width= pInputImage->width;
    height=pInputImage->height;

    //Calculando la imagen de intensidad. El resultado será almacenado
    //en el espacio que fue alocado para tal fin en nuestra estructura
    //pInputImage
    for (j=0;j<height; j++) {
        for (i=0;i<width; i++) {
            pInputImage->pintensity[j*width+i] =
            (unsigned char)(0.299*(double)pInputImage->prgb[3*j*width+3*i]+
                            0.587*(double)pInputImage->prgb[3*j*width+3*i+1]+
                            0.114*(double)pInputImage->prgb[3*j*width+3*i+2]);
        }
    }
}



    //Esta funcion dibuja un segmento lineal sobre
    //la imagen de intensidad
    void geoDrawALinealSegmentOnIntensityImage()
    {
        int x, y, i;
        double alfa;
        int xi, yi, xf, yf;
        int height, width;

        xi=p_parametros->xi;
        yi=p_parametros->yi;
        xf=p_parametros->xf;
        yf=p_parametros->yf;

        height=pInputImage->height;
        width=pInputImage->width;

        //Copiando la imagen de intensidad en la imagen que contendrá el segmento lineal
        for (i=0;i<pInputImage->height*pInputImage->width;i++)
            pInputImage->pdrawnLinealSegmentOnIntensity[i]=pInputImage->pintensity[i];

        //Por cada valor del parámetro alfa se calcula un punto usando las ecuaciones
        //paramétricas de un segmento lineal. Cada punto se pone en 255 (blanco) en la
        //imagen de salida. Alfa varía entre 0.0 y 1.0 en pasos de GEO_ALFA_STEP. Cuando
        //alfa en 0.0 se estaría en la posición inicial del segmento y cuando alfa es
        //1.0 en el punto final del segmento.
        for (alfa=0.0;alfa<=1.0;alfa=alfa+0.001) {
            //x=xi+alfa*(xf-xi)
            x=(int)((double)xi+alfa*((double)xf-(double)xi));
            //y=iy+alfa*(fy-iy)
            y=(int)((double)yi+alfa*((double)yf-(double)yi));

            //Dibujando el punto (x,y) siempre y cuando no esté fuera de la imagen
            if ((y>=0)&&(x>=0)&&(y<height)&&(x<width)) {
                pInputImage->pdrawnLinealSegmentOnIntensity[y*width+x]=255;
            }
        }
    }

    //Esta funcion dibuja un circulo sobre la imagen de
    //intensidad
    void geoDrawACircleOnIntensityImage()
    {
        int x, y, i;
        double angle;
        int cx, cy, r;
        int height, width;

        cx=p_parametros->cx;
        cy=p_parametros->cy;
        r=p_parametros->r;

        height=pInputImage->height;
        width=pInputImage->width;

        //Copiando la imagen de intensidad en la imagen que contendrá
        //el círculo
        for (i=0;i<height*width;i++)
            pInputImage->pdrawnCircleOnIntensity[i]=pInputImage->pintensity[i];

        //Por cada valor del parámetro angle se calcula un punto usando las ecuaciones
        //paramétricas de un círculo. Cada punto se pone en 255 (blanco) en la
        //imagen de salida. angle varía entre 0.0 y 2*PI en pasos de 0.001. Cuando
        //angle en 0.0 se estaría en la posición inicial del círculo (a una distancia
        //radius sobre el eje horizontal del círculo) y cuando angle es 2*PI ya
        //habríamos dado la vuelta y estaríamos cerrando el círculo
        for (angle=0.0;angle<2.0*(double)PI;angle=angle+0.001) {
            //x=cx+r*cos(angle)
            x=(int)((double)cx+(double)r*cos(angle));
            //y=cy+r*sin(angle)
            y=(int)((double)cy+(double)r*sin(angle));

            //Dibujando el punto (x,y) siempre y cuando no esté fuera de la imagen
            if ((y>=0)&&(x>=0)&&(y<height)&&(x<width)) {
                pInputImage->pdrawnCircleOnIntensity[y*width+x]=255;
            }
        }
    }




    //Esta funcion dibuja un triangulo sobre
    //la imagen de intensidad
    void beDrawATriangleOnIntensityImage()
    {
        int segment1x,segment1y, segment2x, segment2y, segment3x, segment3y, i;
        double alfa;
        int v1x, v1y, v2x, v2y, v3x, v3y;
        int height, width;

        v1x=p_parametros->v1x;
        v1y=p_parametros->v1y;
        v2x=p_parametros->v2x;
        v2y=p_parametros->v2y;
        v3x=p_parametros->v3x;
        v3y=p_parametros->v3y;

        height=pInputImage->height;
        width=pInputImage->width;

        //Copiando la imagen de intensidad en la imagen que contendrá el segmento lineal
        for (i=0;i<pInputImage->height*pInputImage->width;i++)
            pInputImage->pdrawnTriangleOnIntensity[i]=pInputImage->pintensity[i];

        //Por cada valor del parámetro alfa se calcula un punto usando las ecuaciones
        //paramétricas de un segmento lineal. Cada punto se pone en 255 (blanco) en la
        //imagen de salida. Alfa varía entre 0.0 y 1.0 en pasos de GEO_ALFA_STEP. Cuando
        //alfa en 0.0 se estaría en la posición inicial del segmento y cuando alfa es
        //1.0 en el punto final del segmento.
        for (alfa=0.0;alfa<=1.0;alfa=alfa+0.001) {

//Segmento horizontal
            //segment1x=v1x+alfa(v2x-v1x)
            segment1x=(int)((double)v1x+alfa*((double)v2x-(double)v1x));
            //segment1x=v1y+alfa
            segment1y=(int)((double)v1y+alfa);//*((double)v2y-(double)v1y));
//Segmento de izquierda traingulo
            //segment2x=v1x+alfa(v2x-v1x)
            segment2x=(int)((double)v1x+alfa*((double)v2x-(double)v1x));
            //segment2x=v1y+alfa(v3y-v1y)
            segment2y=(int)((double)v1y+alfa*((double)v3y-(double)v1y));
//segmento de la izquierda del triangulo
            //segment3x=v2x+alfa
            segment3x=(int)((double)v2x+alfa);
            //segment3x=v3y+alfa(v3y-v2y)
            segment3y=(int)((double)v2y+alfa*((double)v3y-(double)v2y));

            //Dibujando el punto segmento 1 siempre y cuando no esté fuera de la imagen
            if ((segment1y>=0)&&(segment1x>=0)&&(segment1y<height)&&(segment1x<width)) {
                pInputImage->pdrawnTriangleOnIntensity[segment1y*width+segment1x]=255;
            }
            //Dibujando el punto segmento 2 siempre y cuando no esté fuera de la imagen
            if ((segment2y>=0)&&(segment2x>=0)&&(segment2y<height)&&(segment2x<width)) {
                pInputImage->pdrawnTriangleOnIntensity[segment2y*width+segment2x]=255;
            }
            //Dibujando el punto segmento 3 siempre y cuando no esté fuera de la imagen
            if ((segment3y>=0)&&(segment3x>=0)&&(segment3y<height)&&(segment3x<width)) {
                pInputImage->pdrawnTriangleOnIntensity[segment3y*width+segment3x]=255;
            }
        }
    }






        void geoGetMeanImage()
        {
            double *ptempImage;
            unsigned char *pi;
            int x, y, width, height, i;
            height=pInputImage->height;
            width=pInputImage->width;
            pi=pInputImage->pintensity;

            //Allocating memory for temporal image
            ptempImage=(double *)malloc(sizeof(double)*width*height);
            for(i=0;i<width*height;i++) ptempImage[i]=0.0;

            //Getting the mean image
            for(y=1;y<(height-1);y++) {
                for(x=1;x<(width-1);x++) {
                    ptempImage[y*width+x]= ((double)pi[y*width+x]+
                                            (double)pi[y*width+x-1]+
                                            (double)pi[y*width+x+1]+
                                            (double)pi[(y-1)*width+x]+
                                            (double)pi[(y-1)*width+x-1]+
                                            (double)pi[(y-1)*width+x+1]+
                                            (double)pi[(y+1)*width+x]+
                                            (double)pi[(y+1)*width+x-1]+
                                            (double)pi[(y+1)*width+x+1])/9.0;
                }
            }

                //Saving Image
                for(i=0;i<height*width;i++) {
                    //Copying exact mean image
                    pInputImage->pmeanImage[i]=ptempImage[i];
                    //Copying also an unsigned char version of the mean image for bmp file storage
                    //or OpenCV windows displays
                    if(fabs(ptempImage[i])>=255.0){
                    pInputImage->pmeanImage_uc[i]=255;
                    }
                    else{
                    pInputImage->pmeanImage_uc[i]=(unsigned char)ptempImage[i];
                    }
                }
                //Freeing memory
                free(ptempImage);
            }

            void geoGetVarianceImage()
            {
                double *ptempImage, *pm, mean2;
                unsigned char *pi;
                int width, height, i, j;
                height=pInputImage->height;
                width=pInputImage->width;
                pi=pInputImage->pintensity;
                pm=pInputImage->pmeanImage;

                //Allocating memory for temporal image
                ptempImage=(double *)malloc(sizeof(double)*width*height);
                for(i=0;i<width*height;i++) ptempImage[i]=0.0;

                //Getting the variance image
                for(j=1;j<(height-1);j++) {
                    for(i=1;i<(width-1);i++) {
                        mean2=pm[j*width+i]*pm[j*width+i];
                        ptempImage[j*width+i]= ((double)pi[j*width+i]*(double)pi[j*width+i]-mean2+
                                                (double)pi[j*width+i-1]*(double)pi[j*width+i-1]-mean2+
                                                (double)pi[j*width+i+1]*(double)pi[j*width+i+1]-mean2+
                                                (double)pi[(j-1)*width+i]*(double)pi[(j-1)*width+i]-mean2+
                                                (double)pi[(j-1)*width+i-1]*(double)pi[(j-1)*width+i-1]-mean2+
                                                (double)pi[(j-1)*width+i+1]*(double)pi[(j-1)*width+i+1]-mean2+
                                                (double)pi[(j+1)*width+i]*(double)pi[(j+1)*width+i]-mean2+
                                                (double)pi[(j+1)*width+i-1]*(double)pi[(j+1)*width+i-1]-mean2+
                                                (double)pi[(j+1)*width+i+1]*(double)pi[(j+1)*width+i+1]-mean2)/8.0;
                    }
                }
                //Copying result in pInputImage
                    for(i=0;i<height*width;i++) {
                        //Copying exact variance image
                        pInputImage->pvarianceImage[i]=ptempImage[i];
                        //Copying also an insigned char version of the variance image for bmp
                        //file storage or Open windows displays
                        if(fabs(ptempImage[i])>=255.0){
                        pInputImage->pvarianceImage_uc[i]=255;
                        }
                        else{
                        pInputImage->pvarianceImage_uc[i]=(unsigned char)ptempImage[i];
                        }
                    }
                    //Freeing memory
                    free(ptempImage);
                }






void geoLeerParametrosDeControlDeArchivoDeTexto()
{
    FILE *archivo;
    char d1[256], d2[256], d3[256];
    int res;

    printf("Leyendo los datos de entrada:\n");

    //Abriendo archivo en mode de lectura
    char nombreDeArchivo[256]="current_control_parameters.txt";
    archivo = fopen(nombreDeArchivo, "r");
    if (!archivo) {
        printf("No se pudo abrir el archivo: current_control_parameters.txt\n");
        exit(1);
    }

    //Leyendo datos linea por linea

    //Brincando la primera y segunda lineas
    res=fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    res=fscanf(archivo, "\n");
    numeroDeDatosLeidos++;

    printf("  Dimensiones de las imagenes\n");

    //Leyendo width
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->width=(int)atof(d2);
    printf("   width: %d\n", p_parametros->width);
    numeroDeDatosLeidos++;

    //Leyendo height
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->height=(int)atof(d2);
    printf("   height: %d\n", p_parametros->height);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    res=fscanf(archivo, "\n");

    printf("  Imagen de entrada y directorio de salida\n");

    //Leyendo path y nombre de imagen de entrada
    res=fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    strcpy(p_parametros->pathAndInputImageFileName,d3);
    printf("   imagen de entrada: %s\n", p_parametros->pathAndInputImageFileName);
    numeroDeDatosLeidos++;

    //Leyendo directorio de salida
    res=fscanf(archivo, "%s %s %s\n", d1, d2, d3);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    strcpy(p_parametros->pathOfOutputDirectory,d3);
    printf("   directorio de salida: %s\n", p_parametros->pathOfOutputDirectory);
    numeroDeDatosLeidos++;

    res=fscanf(archivo, "\n");

    printf("  Punto inicial y punto final\n");

    //Leyendo xi
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->xi=(int)atof(d2);
    printf("   xi: %d\n", p_parametros->xi);
    numeroDeDatosLeidos++;

    //Leyendo yi
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->yi=(int)atof(d2);
    printf("   yi: %d\n", p_parametros->yi);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    res=fscanf(archivo, "\n");

    //Leyendo xf
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->xf=(int)atof(d2);
    printf("   xf: %d\n", p_parametros->xf);
    numeroDeDatosLeidos++;

    //Leyendo yf
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->yf=(int)atof(d2);
    printf("   yf: %d\n", p_parametros->yf);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    res=fscanf(archivo, "\n");

    printf("  Centro del circulo\n");

    //Leyendo cx
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->cx=(int)atof(d2);
    printf("   cx: %d\n", p_parametros->cx);
    numeroDeDatosLeidos++;

    //Leyendo cy
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->cy=(int)atof(d2);
    printf("   cy: %d\n", p_parametros->cy);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    res=fscanf(archivo, "\n");

    printf("  Radio del circulo\n");

    //Leyendo r
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->r=(int)atof(d2);
    printf("   r: %d\n", p_parametros->r);
    numeroDeDatosLeidos++;

    printf("  Vertices del triangulo\n");

    //Leyendo v1x
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->v1x=(int)atof(d2);
    printf("   v1x: %d\n", p_parametros->v1x);
    numeroDeDatosLeidos++;

    //Leyendo v1y
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->v1y=(int)atof(d2);
    printf("   v1y: %d\n", p_parametros->v1y);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    res=fscanf(archivo, "\n");

    //Leyendo v2x
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->v2x=(int)atof(d2);
    printf("   v2x: %d\n", p_parametros->v2x);
    numeroDeDatosLeidos++;

    //Leyendo v2y
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->v2y=(int)atof(d2);
    printf("   v2y: %d\n", p_parametros->v2y);
    numeroDeDatosLeidos++;

    //Brincando linea de texto
    res=fscanf(archivo, "\n");

    //Leyendo v3x
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->v3x=(int)atof(d2);
    printf("   v3x: %d\n", p_parametros->v3x);
    numeroDeDatosLeidos++;

    //Leyendo v3y
    res=fscanf(archivo, "%s %s\n", d1, d2);
    if (res==0) {printf("Error leyendo dato No. %d de archivo de parametros de control\n", numeroDeDatosLeidos); exit(0);}
    p_parametros->v3y=(int)atof(d2);
    printf("   v3y: %d\n", p_parametros->v3y);
    numeroDeDatosLeidos++;


    printf("  Numero de datos leidos: %d\n", numeroDeDatosLeidos);

    //Cerrando archivo
    fclose(archivo);
}









void getHistogramAndProbabilityDensityFunctionOfIntensityValues()
{

      int w, h, N, i, intensityValue, his[256];
      double p[256];
      h=pInputImage->height;
      w=pInputImage->width;
      N=h*w;

      //Initialization
      for(i=0;i<256;i++) {
          his[i]=0;
          p[i]=0.0;
      }

      //Getting the histogram of the intensity values
      for(i=0;i<w*h;i++) {
          intensityValue=pInputImage->pintensity[i];
          his[intensityValue]=his[intensityValue]+1;
      }

      //Printing histogram in terminal
      for(i=0;i<256;i++) printf("Histograma[%d]: %d\n", i, his[i]);

      //Getting the probability density function
      for(i=0;i<256;i++) {
          p[i]=(double)his[i]/(double)N;
      }
      printf("Densidad de la probibilidad:\n");
      for(i=0;i<256;i++) printf("p[%d]: %f\n", i, p[i]);
      //Saving results
      for(i=0;i<256;i++) {
          pInputImage->histograma[i]=his[i];
          pInputImage->p[i]=p[i];
      }
}

void getMeanMeanOfSquaresAndVarianceOfIntensityValues()
{
  int i;
  double m, ms, var;

  //Getting the mean of the intensity values
  m=0.0;
  for(i=0;i<256;i++) {
      m=m+(double)i*pInputImage->p[i];
  }
  printf("Valor medio: %f\n", m);
  pInputImage->medidas_estadisticas[0]=m;

  //Getting the mean of squares of the intensity values
  ms=0.0;
      for(i=0;i<256;i++) {
      ms=ms+(double)i*i*pInputImage->p[i];
  }
  printf("Valor medio cuadratico: %f\n", ms);
  pInputImage->medidas_estadisticas[1]=ms;

  //Getting the variance of the intensity values
  var=0.0;
  for(i=0;i<256;i++) {
      var=var+((double)i-m)*((double)i-m)*pInputImage->p[i];
  }
  printf("Varianza: %f\n", var);
  pInputImage->medidas_estadisticas[2]=var;
}


//Saving results
void beSaveResultsOfProbabilityInTextFile()
{
  FILE *archivo;
  int i;

  //Abriendo archivo en modo de escritura
  char nombreDeArchivo[256]="output/resultados.txt";
  archivo = fopen(nombreDeArchivo, "w");
  if (!archivo) {
      printf("No se pudo abrir el archivo: resultados.txt\n");
      exit(1);
  }

  //Guardando histograma
  fprintf(archivo, "Histograma:\n");
  for(i=0;i<256;i++) {
      fprintf(archivo, "Histograma[%d]: %d\n",i,pInputImage->histograma[i]);
  }
    //Guardando densidad de probabilidad
    fprintf(archivo, "Densidad de la probabilidad:\n");
    for(i=0;i<256;i++) {
        fprintf(archivo, "p[%d]: %f\n",i,pInputImage->p[i]);
    }
    fprintf(archivo, "\n"); //linea en blanco
    fprintf(archivo, "Valor medio = %f\n",pInputImage->medidas_estadisticas[0]);
    fprintf(archivo, "Valor medio cuadratico = %f\n",pInputImage->medidas_estadisticas[1]);
    fprintf(archivo, "Valor de la varianza = %f\n",pInputImage->medidas_estadisticas[2]);
    fclose(archivo);
    }






// esta función implementa el algoritmo de KITTLER
void umbralizacionkittler()
{
int i;
int iterador_para_calculo_adecuado_umbral;
double tope_funcion_verosimi,varianza1, varianza2,funcion_de_verosimilutud;
int umbral_optimo;
double m1_sumatoria_antes_De_dividir, m2_sumatoria_antes_De_dividir;
double c1, c2, N, N2;
int almacena_umbral_compara, almacena_comparacion_umbral_op;
double varianza1_sumatoria_antes_de_diviir, varianza2_sumatoria_antes_de_diviir;
double calculo_varianza1_con_umbral_optimo, calculo_varianza2_con_umbral_optimo;
double calculo_m1_con_umbral_optimo, calculo_m2_con_umbral_optimo;
int width, height;



tope_funcion_verosimi=3400000.00;
umbral_optimo=0;






for(iterador_para_calculo_adecuado_umbral=0;iterador_para_calculo_adecuado_umbral<=255;iterador_para_calculo_adecuado_umbral++){
  c1=0.0;
  c2=0.0;
  //calculo de c1y c2

for(i=0;i<=iterador_para_calculo_adecuado_umbral;i++){
  c1=c1+(double)pInputImage->p[i];
}
//calculo de c2 medinte modelo de poblacion mixta
  c2=1-c1;

  almacena_umbral_compara=iterador_para_calculo_adecuado_umbral+1;
  m1_sumatoria_antes_De_dividir=0.0;
  m2_sumatoria_antes_De_dividir=0.0;

  for(i=0;i<=iterador_para_calculo_adecuado_umbral;i++){
      m1_sumatoria_antes_De_dividir=m1_sumatoria_antes_De_dividir+(double)i*pInputImage->p[i];
  }
  pInputImage->m1=m1_sumatoria_antes_De_dividir/c1;


  for(i=almacena_umbral_compara;i<=255;i++){
      m2_sumatoria_antes_De_dividir=m2_sumatoria_antes_De_dividir+(double)i*pInputImage->p[i];
  }
  pInputImage->m2=m2_sumatoria_antes_De_dividir/c2;
  varianza1_sumatoria_antes_de_diviir=0.0;
  varianza2_sumatoria_antes_de_diviir=0.0;

  for(i=0;i<=iterador_para_calculo_adecuado_umbral;i++)
  {
      varianza1_sumatoria_antes_de_diviir=varianza1_sumatoria_antes_de_diviir+((double)i-pInputImage->m1)*((double)i-pInputImage->m1)*pInputImage->p[i];
  }
  pInputImage->varianza1=varianza1_sumatoria_antes_de_diviir/c1;

  for(i=almacena_umbral_compara;i<=255;i++){
      varianza2_sumatoria_antes_de_diviir=varianza2_sumatoria_antes_de_diviir+((double)i-pInputImage->m2)*((double)i-pInputImage->m2)*pInputImage->p[i];
  }
  pInputImage->varianza2=varianza2_sumatoria_antes_de_diviir/c2;
  funcion_de_verosimilutud=0.0;
  if(c1!=0.00 && pInputImage->varianza1!=0.00 && c2!=0.00 && pInputImage->varianza2!=0.00){
  varianza1=(c1*log(c1))+(c2*log(c2));
  varianza2=(c1*log(pInputImage->varianza1))+(c2*log(pInputImage->varianza2));
  N=(pInputImage->width)*(pInputImage->height);
  N2=N/2;

  funcion_de_verosimilutud=N*varianza1-(N2*log(2*PI))-(N2*varianza2)-N2;
  }

  if(funcion_de_verosimilutud!=0){
  if(fabs(funcion_de_verosimilutud)<tope_funcion_verosimi){
      tope_funcion_verosimi=fabs(funcion_de_verosimilutud);
      umbral_optimo=iterador_para_calculo_adecuado_umbral;
  }
}

}
  pInputImage->funcion_vero=umbral_optimo;

  for(i=0;i<=umbral_optimo;i++){
  pInputImage->c1=pInputImage->c1+(double)pInputImage->p[i];
}

  pInputImage->c2=1-pInputImage->c1;

  almacena_comparacion_umbral_op=umbral_optimo+1;

  calculo_m1_con_umbral_optimo=0.0;
  calculo_m2_con_umbral_optimo=0.0;
  for(i=0;i<=umbral_optimo;i++){
      calculo_m1_con_umbral_optimo=calculo_m1_con_umbral_optimo+(double)i*pInputImage->p[i];
  }
  pInputImage->m1=calculo_m1_con_umbral_optimo/pInputImage->c1;


  for(i=almacena_comparacion_umbral_op;i<=255;i++){
      calculo_m2_con_umbral_optimo=calculo_m2_con_umbral_optimo+(double)i*pInputImage->p[i];
  }
  pInputImage->m2=calculo_m2_con_umbral_optimo/pInputImage->c2;
  calculo_varianza1_con_umbral_optimo=0.0;
  calculo_varianza2_con_umbral_optimo=0.0;


  for(i=0;i<=umbral_optimo;i++)
  {
      calculo_varianza1_con_umbral_optimo=calculo_varianza1_con_umbral_optimo+((double)i-pInputImage->m1)*((double)i-pInputImage->m1)*pInputImage->p[i];
  }
  pInputImage->varianza1=calculo_varianza1_con_umbral_optimo/pInputImage->c1;

  for(i=almacena_comparacion_umbral_op;i<=255;i++){
      calculo_varianza2_con_umbral_optimo=calculo_varianza2_con_umbral_optimo+((double)i-pInputImage->m2)*((double)i-pInputImage->m2)*pInputImage->p[i];
  }
  pInputImage->varianza2=calculo_varianza2_con_umbral_optimo/pInputImage->c2;


  //imprimeinto en terminal los optimos


    printf("--------------------------\n");
    printf("Datos obtenidos para umbralización Kittler\n");
    printf("--------------------------\n");

    printf("\nc1: %f\n", pInputImage->c1);
    printf("\nc2: %f\n", pInputImage->c2);
    printf("\nvarianza1: %f\n", pInputImage->varianza1);
    printf("\nvarianza2: %f\n", pInputImage->varianza2);

    printf("\nm1: %f\n", pInputImage->m1);
    printf("\nm2: %f\n", pInputImage->m2);
    printf("\nthOp: %i\n", pInputImage->funcion_vero);



    width=pInputImage->width;
    height=pInputImage->height;
    //imgen segmentada
    for(i=0;i<width*height;i++)
    {
        if(pInputImage->pintensity[i]>=umbral_optimo)
        {
            pInputImage->pimagensegmentada[i]=0;
        }
        else {
            pInputImage->pimagensegmentada[i]=255;
        }
    }

}


void Save_umbralizacion_results_in_text_file()
{

  FILE *archivo;
  int i;

  //Abriendo archivo en modo de escritura
  char nombreDeArchivo[256]="output/resultadosOptimosSegunKittler.txt";
  archivo = fopen(nombreDeArchivo, "w");
  if (!archivo) {
      printf("No se pudo abrir el archivo: resultadosOptimosSegunKittler.txt\n");
      exit(1);
  }
  fprintf(archivo, "\n"); //linea en blanco

  fprintf(archivo, "---------------------------\n");
  fprintf(archivo, "\nKittler umbralización\n");
  fprintf(archivo, "---------------------------\n");

  fprintf(archivo, "\nc1: %f\n", pInputImage->c1);
  fprintf(archivo, "\nc2: %f\n", pInputImage->c2);
  fprintf(archivo, "\nvarianza1: %f\n", pInputImage->varianza1);
  fprintf(archivo, "\nvarianza2: %f\n", pInputImage->varianza2);

  fprintf(archivo, "\nm1: %f\n", pInputImage->m1);
  fprintf(archivo, "\nm2: %f\n", pInputImage->m2);
  fprintf(archivo, "\nthOp: %i\n", pInputImage->funcion_vero);


  fprintf(archivo, "\n"); //linea en blanco





  //Guardando c1
  fprintf(archivo, "Variables c1:\n");
  for(i=0;i<256;i++) {
      fprintf(archivo, "c1[%d]: %f\n",i,pInputImage->c1);
  }
  fprintf(archivo, "\n"); //linea en blanco

  //Guardando c2
  fprintf(archivo, "Variables c2:\n");
  for(i=0;i<256;i++) {
      fprintf(archivo, "c2[%d]: %f\n",i,pInputImage->c2);
  }
  fprintf(archivo, "\n"); //linea en blanco
  //Guardando m1
  fprintf(archivo, "Variables m1:\n");
  for(i=0;i<256;i++) {
      fprintf(archivo, "m1[%d]: %f\n",i,pInputImage->m1);
  }


  fprintf(archivo, "\n"); //linea en blanco
  //Guardando m2
  fprintf(archivo, "Variables m2:\n");
  for(i=0;i<256;i++) {
      fprintf(archivo, "m2[%d]: %f\n",i,pInputImage->m2);
  }
  fprintf(archivo, "\n"); //linea en blanco
  //Guardando varianza1
  fprintf(archivo, "Variables Varianza1:\n");
  for(i=0;i<256;i++) {
      fprintf(archivo, "varianza1[%d]: %f\n",i,pInputImage->varianza1);
  }
  fprintf(archivo, "\n"); //linea en blanco

  //Guardando varianza2
  fprintf(archivo, "Variables Varianza2:\n");
  for(i=0;i<256;i++) {
      fprintf(archivo, "varianza2[%d]: %f\n",i,pInputImage->varianza2);
  }

  fprintf(archivo, "\n"); //linea en blanco

  //Guardando funcion de verosimilitud
  fprintf(archivo, "Variables Función de verosimilitud:\n");
  for(i=0;i<256;i++) {
      fprintf(archivo, "verosimilitud[%d]: %d\n",i,pInputImage->funcion_vero);
  }

  fprintf(archivo, "\n"); //linea en blanco


  fclose(archivo);


  /*
    FILE *archivo;
    int i;

    char salida[256];
    strcpy(salida, "output/resultadosOptimosSegunKittler.txt");
 //copia el camino del archivo en una nueva variable

    //Abriendo archivo en modo de escritura
    archivo = fopen(salida, "w"); //crea el archivo de resultados, se concatena la ruta
    if (!archivo) {
        printf("No se pudo abrir el archivo: resultados.txt\n");
        exit(1);
    }
    fprintf(archivo, "Kittler umbralización\n");
    fprintf(archivo, "thOp: %i\n", pInputImage->funcion_vero);
    fprintf(archivo, "c1: %f\nm1: %f\nVarianza1: %f\n", pInputImage->c1, pInputImage->m1, pInputImage->varianza1);
    fprintf(archivo, "c2: %f\nm2: %f\nVarianza2: %f\n", pInputImage->c2, pInputImage->m2, pInputImage->varianza2);

*/

}



    //****************************************
    //****************************************
    //** FUNCIONES DE LECTURA Y ESCRITURA   **
    //** DE IMAGENES                        **
    //****************************************
    //****************************************



struct BMPHeader
{
    char bfType[3];       /* "BM" */
    int bfSize;           /* Size of file in bytes */
    int bfReserved;       /* set to 0 */
    int bfOffBits;        /* Byte offset to actual bitmap data (= 54) */
    int biSize;           /* Size of BITMAPINFOHEADER, in bytes (= 40) */
    int biWidth;          /* Width of image, in pixels */
    int biHeight;         /* Height of images, in pixels */
    short biPlanes;       /* Number of planes in target device (set to 1) */
    short biBitCount;     /* Bits per pixel (24 in this case) */
    int biCompression;    /* Type of compression (0 if no compression) */
    int biSizeImage;      /* Image size, in bytes (0 if no compression) */
    int biXPelsPerMeter;  /* Resolution in pixels/meter of display device */
    int biYPelsPerMeter;  /* Resolution in pixels/meter of display device */
    int biClrUsed;        /* Number of colors in the color table (if 0, use
                           maximum allowed by biBitCount) */
    int biClrImportant;   /* Number of important colors.  If 0, all colors
                           are important */
};



//Esta funcion lee imagen RGB de archivo en formato BMP
void readRGBImageFromBMPFile(char *filename)
{
    FILE *fd;
    int width, height;
    int i, j;
    int n;

    fd = fopen(filename, "rb");
    if (fd == NULL)
    {
        printf("Error: fopen failed\n");
        return;
    }

    unsigned char header[54];

    // Read header
    n=fread(header, sizeof(unsigned char), 54, fd);
    if (n==0) {printf("No se pudieron leer datos\n"); exit(0);}

    // Capture dimensions
    width = *(int*)&header[18];
    height = *(int*)&header[22];

    int padding = 0;

    // Calculate padding
    while ((width * 3 + padding) % 4 != 0)
    {
        padding++;
    }

    // Compute new width, which includes padding
    int widthnew = width * 3 + padding;

    // Allocate temporary memory to read widthnew size of data
    unsigned char* data = (unsigned char *)malloc(widthnew * sizeof (unsigned int));

    // Read row by row of data and remove padded data.
    for (i = 0; i<height; i++)
    {
        // Read widthnew length of data
        n=fread(data, sizeof(unsigned char), widthnew, fd);
        if (n==0) {printf("No se pudieron leer datos\n"); exit(0);}

        // Retain width length of data, and swizzle RB component.
        // BMP stores in BGR format, my usecase needs RGB format
        for (j = 0; j < width * 3; j += 3)
        {
            int index = (i * width * 3) + (j);
            pInputImage->prgb[index + 0] = data[j + 2];
            pInputImage->prgb[index + 1] = data[j + 1];
            pInputImage->prgb[index + 2] = data[j + 0];
        }
    }

    free(data);
    fclose(fd);
}




    //Esta funcion almacena una imagen de intensidad en
    //archivo en formato YUV400
    void geoSaveIntensityImageIn_YUV400_file(unsigned char *pintensity, char* filename)
    {
        int numread, j, jj, i;
        FILE *fpointer;
        int width, height;
        unsigned char *ptempImage;

        width=pInputImage->width;
        height=pInputImage->height;






        //Imagen de uso local y temporal
        ptempImage = (unsigned char *)malloc(sizeof(unsigned char)*width*height);

        printf("Saving unsigned char image in: %s\n", filename);

        //Abriendo archivo
        fpointer = fopen(filename, "w");
        if (fpointer == NULL) {
            printf("could not read the file: %s\n", filename);
            exit(0);
        }





        //Cambiando posición del sistema de coordenadas de la equina inferior
        //izquierda a la esquina superior izquierda.
        for (i=0;i<width*height;i++) ptempImage[i]=0;
        jj=0;
        for (j=height-1;j>=0;j--) {
            for (i=0;i<width;i++) {
                ptempImage[jj*width+i]= pintensity[j*width+i];
            }
            jj++;
        }





        //Almacenando valores de intensidad en archivo
        numread = (int)fwrite(ptempImage, sizeof(unsigned char), (unsigned int)(height*width), fpointer);

        if (numread==0) {
            printf("Por alguna razon no se pudo escribir dato alguno en archivo\n");
            exit(0);
        }

        fclose(fpointer);
        free(ptempImage);
    }









//Esta funcion almacena una imagen de intensidad en
//archivo en formato BMP
int SaveIntensityImageIn_BMP_file(unsigned char *pintensity, char *filename)
{
    int i, j, jj, ipos;
    int bytesPerLine;
    unsigned char *line;
    unsigned char *ptempImage;
    int height, width;

    height=pInputImage->height;
    width=pInputImage->width;

    FILE *file;
    struct BMPHeader bmph;

    /* The length of each line must be a multiple of 4 bytes */

    bytesPerLine = (3 * (width + 1) / 4) * 4;

    strcpy(bmph.bfType, "BM");
    bmph.bfOffBits = 54;
    bmph.bfSize = bmph.bfOffBits + bytesPerLine * height;
    bmph.bfReserved = 0;
    bmph.biSize = 40;
    bmph.biWidth = width;
    bmph.biHeight = height;
    bmph.biPlanes = 1;
    bmph.biBitCount = 24;
    bmph.biCompression = 0;
    bmph.biSizeImage = bytesPerLine * height;
    bmph.biXPelsPerMeter = 0;
    bmph.biYPelsPerMeter = 0;
    bmph.biClrUsed = 0;
    bmph.biClrImportant = 0;

    file = fopen (filename, "wb");
    if (file == NULL) return(0);

    fwrite(&bmph.bfType, 2, 1, file);
    fwrite(&bmph.bfSize, 4, 1, file);
    fwrite(&bmph.bfReserved, 4, 1, file);
    fwrite(&bmph.bfOffBits, 4, 1, file);
    fwrite(&bmph.biSize, 4, 1, file);
    fwrite(&bmph.biWidth, 4, 1, file);
    fwrite(&bmph.biHeight, 4, 1, file);
    fwrite(&bmph.biPlanes, 2, 1, file);
    fwrite(&bmph.biBitCount, 2, 1, file);
    fwrite(&bmph.biCompression, 4, 1, file);
    fwrite(&bmph.biSizeImage, 4, 1, file);
    fwrite(&bmph.biXPelsPerMeter, 4, 1, file);
    fwrite(&bmph.biYPelsPerMeter, 4, 1, file);
    fwrite(&bmph.biClrUsed, 4, 1, file);
    fwrite(&bmph.biClrImportant, 4, 1, file);

    line = (unsigned char*) malloc(bytesPerLine);
    if (line == NULL)
    {
        fprintf(stderr, "Can't allocate memory for BMP file.\n");
        return(0);
    }

    //Cambiando posición del sistema de coordenadas de la equina inferior
    //izquierda a la esquina superior izquierda.
    ptempImage = (unsigned char *)malloc(sizeof(unsigned char)*width*height);
    for (i=0;i<width*height;i++) ptempImage[i]=0;
    jj=0;
    for (j=height-1;j>=0;j--) {
        for (i=0;i<width;i++) {
            ptempImage[jj*width+i]= pintensity[j*width+i];
        }
        jj++;
    }

    for (i = height - 1; i >= 0; i--)
    {
        for (j = 0; j < width; j++)
        {
            ipos = (width * i + j);
            line[3*j] = ptempImage[ipos];
            line[3*j+1] = ptempImage[ipos];
            line[3*j+2] = ptempImage[ipos];
        }
        fwrite(line, bytesPerLine, 1, file);
    }

    free(line);
    fclose(file);

    free(ptempImage);

    return(1);
}

//Esta funcion almacena una imagen RGB en
//archivo en formato BMP
int SaveRGBImageIn_BMP_file(unsigned char *prgb, char *filename)
{
    int i, j, jj, ipos;
    int bytesPerLine;
    unsigned char *line;
    unsigned char *ptempImage;
    int height, width;

    height=pInputImage->height;
    width=pInputImage->width;

    FILE *file;
    struct BMPHeader bmph;

    /* The length of each line must be a multiple of 4 bytes */

    bytesPerLine = (3 * (width + 1) / 4) * 4;

    strcpy(bmph.bfType, "BM");
    bmph.bfOffBits = 54;
    bmph.bfSize = bmph.bfOffBits + bytesPerLine * height;
    bmph.bfReserved = 0;
    bmph.biSize = 40;
    bmph.biWidth = width;
    bmph.biHeight = height;
    bmph.biPlanes = 1;
    bmph.biBitCount = 24;
    bmph.biCompression = 0;
    bmph.biSizeImage = bytesPerLine * height;
    bmph.biXPelsPerMeter = 0;
    bmph.biYPelsPerMeter = 0;
    bmph.biClrUsed = 0;
    bmph.biClrImportant = 0;

    file = fopen (filename, "wb");
    if (file == NULL) return(0);

    fwrite(&bmph.bfType, 2, 1, file);
    fwrite(&bmph.bfSize, 4, 1, file);
    fwrite(&bmph.bfReserved, 4, 1, file);
    fwrite(&bmph.bfOffBits, 4, 1, file);
    fwrite(&bmph.biSize, 4, 1, file);
    fwrite(&bmph.biWidth, 4, 1, file);
    fwrite(&bmph.biHeight, 4, 1, file);
    fwrite(&bmph.biPlanes, 2, 1, file);
    fwrite(&bmph.biBitCount, 2, 1, file);
    fwrite(&bmph.biCompression, 4, 1, file);
    fwrite(&bmph.biSizeImage, 4, 1, file);
    fwrite(&bmph.biXPelsPerMeter, 4, 1, file);
    fwrite(&bmph.biYPelsPerMeter, 4, 1, file);
    fwrite(&bmph.biClrUsed, 4, 1, file);
    fwrite(&bmph.biClrImportant, 4, 1, file);

    line = (unsigned char*) malloc(bytesPerLine);
    if (line == NULL)
    {
        fprintf(stderr, "Can't allocate memory for BMP file.\n");
        return(0);
    }

    //Cambiando posición del sistema de coordenadas de la equina inferior
    //izquierda a la esquina superior izquierda.
    ptempImage = (unsigned char *)malloc(sizeof(unsigned char)*width*height*3);
    for (i=0;i<width*height*3;i++) ptempImage[i]=0;
    jj=0;
    for (j=height-1;j>=0;j--) {
        for (i=0;i<width;i++) {
            ptempImage[jj*3*width+3*i]= prgb[j*3*width+3*i];
            ptempImage[jj*3*width+3*i+1]= prgb[j*3*width+3*i+1];
            ptempImage[jj*3*width+3*i+2]= prgb[j*3*width+3*i+2];
        }
        jj++;
    }

    for (i = height - 1; i >= 0; i--)
    {
        for (j = 0; j < width; j++)
        {
            ipos = 3*(width * i + j);
            line[3*j] = ptempImage[ipos+2];
            line[3*j+1] = ptempImage[ipos+1];
            line[3*j+2] = ptempImage[ipos];
        }
        fwrite(line, bytesPerLine, 1, file);
    }

    free(line);
    fclose(file);

    free(ptempImage);

    return(1);
}

//***************COMIENZO PASO (5)***********
//*******************************************
//*********De codeBlocks a ROS***************
//*******************************************

//AGREGAR:

//Esta función se ejecutará automáticamente cada vez que un mensaje sea recibido
//a través del tópico "/usb_cam/image_raw".
void geoFunctionToHandlePublishedImage(const sensor_msgs::ImageConstPtr& msg)
  {
    //Extrayendo la imagen rgb del mensaje recibido
    try
    {
      //cv_prt es un puntero a una estructura ROS que
      //contiene un puntero a otra estructura OpenCV que
      //a su vez contiene un puntero a la imagen rgb
      //recibida.
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

      //cv_prt_display es una copia de cv_prt. Esta copia
      //se usará únicamente para visualización
      cv_ptr_display = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //Desplegando la imagen rgb recibida
    cv::imshow(windowName0, cv_ptr_display->image);
    cvWaitKey(30);

    //Extrayendo la imagen rgb recibida de la estructura OpenCV y
    //copiando su contenido a nuestra estructura global "pInputImage".
    geoCopyTheRGBImageDataFromTheOpencvImageStructureToOurpInputImageStructure();
    //El puntero a la imagen rgb recibida se puede acceder desde
    //nuestra estructura como sigue:
    //pInputImage->prgb

    //*********************************************
    //*********************************************
    //*********************************************
    //*********************************************
    geoInsertYourCodeHere();
    //*********************************************
    //*********************************************
    //*********************************************
    //*********************************************

    contadorDeImagenesRecibidas++;
}

void geoCopyTheRGBImageDataFromTheOpencvImageStructureToOurpInputImageStructure()
{
    unsigned char *ptr, *ptempRgbImage, *pintensity, *prgb;
    int i,j, jj, width, height;

    //Getting image dimentions
    width=cv_ptr->image.cols;
    height = cv_ptr->image.rows;

    if (width!=pInputImage->width || height!=pInputImage->height) {
       printf("\n");
       printf("**************************************************************\n");
       printf("** Error, el ancho (width) y el alto (height) de las imagenes\n");
       printf("** capturadas por la camara es %d, %d, respectivamente. \n", width, height);
       printf("** \n");
       printf("** Cambiar valores en el archivo de paramatros de control\n");
       printf("**************************************************************\n");
       printf("\n");
       exit(0);
    }

    pInputImage->width=width; //hight is extracted from the messages
    pInputImage->height=height; //width is extracted from the messages

    //Renaming pointer names to simplify the code
    prgb= pInputImage->prgb;
    ptempRgbImage = (unsigned char*)malloc(sizeof(unsigned char)*(unsigned int)(width*height*3));
    for (i=0;i<width*height*3;i++) ptempRgbImage[i]=0;
    pintensity=pInputImage->pintensity;

    //Converting opencv image form BGR to RGB
    cv::cvtColor(cv_ptr->image ,cv_ptr->image, CV_BGR2RGB);

    //Extracting RGB image from opencv image structure
    for (i=0;i<height; i++) {
        ptr=(unsigned char *)(cv_ptr->image.data+i*cv_ptr->image.step);
        for (j=0;j<width; j++) {
	     prgb[i*width*3+j*3]=ptr[3*j];
	     prgb[i*width*3+j*3+1]=ptr[3*j+1];
	     prgb[i*width*3+j*3+2]=ptr[3*j+2];
        }
    }

    //Changing the image coordinate from the upper left corner
    //to the lower left corner
    jj=0;
    for (j=height-1;j>=0;j--) {
       for (i=0;i<width;i++) {
            ptempRgbImage[jj*3*width+i*3]=prgb[j*3*width+i*3];
            ptempRgbImage[jj*3*width+i*3+1]=prgb[j*3*width+i*3+1];
            ptempRgbImage[jj*3*width+i*3+2]=prgb[j*3*width+i*3+2];
       }
       jj++;
    }
    for (i=0;i<width*height*3;i++) prgb[i]=ptempRgbImage[i];

    //Getting the intensity image
    for (j=0;j<height; j++) {
       for (i=0;i<width; i++) {
            pintensity[j*width+i] =
                        (unsigned char)(0.299*(double)prgb[3*j*width+3*i]+
	                                0.587*(double)prgb[3*j*width+3*i+1]+
                                        0.114*(double)prgb[3*j*width+3*i+2]);
       }
    }

    //Converting back opencv image from RGB to BGR
    cv::cvtColor(cv_ptr->image ,cv_ptr->image, CV_RGB2BGR);

    free(ptempRgbImage);

}

void geoDisplayInWindowIntensityImage(const unsigned char *pIntensityImage, char windowName[256])
{
	unsigned char *ptr, *ptempImage;
	int i,j, jj;
        int width, height;

        //Renaming to facilitate code
        width= pInputImage->width;
        height=pInputImage->height;

        //Allocating temporal rgb image
	ptempImage = (unsigned char*)malloc(sizeof(unsigned char)*(unsigned int)(width*height*3));
	for (i=0;i<width*height*3;i++) ptempImage[i]=0;

	//Copying the intensity value of each pixel in each of the rgb values of a temporal rgb
        //image and changing the image coordinate system from the lower left corner to the
        //upper left corner
	for (i=0;i<width*height*3;i++) ptempImage[i]=0;
	jj=0;
	for (j=height-1;j>=0;j--) {
	  for (i=0;i<width;i++) {
		  /*R*/ ptempImage[jj*3*width+i*3]=  /*R*/ pIntensityImage[j*width+i];
		  /*G*/ ptempImage[jj*3*width+i*3+1]=/*G*/ pIntensityImage[j*width+i];
		  /*B*/ ptempImage[jj*3*width+i*3+2]=/*B*/ pIntensityImage[j*width+i];
	  }
	  jj++;
	}
        //Copying the rgb data of the temporal rgb image to the rgb data of the
        //opencv image structure
	for (i=0;i<height; i++) {
	   ptr=(unsigned char *)(cv_ptr_display->image.data+i*cv_ptr_display->image.step);
	   for (j=0;j<width; j++) {
			  ptr[3*j]=ptempImage[i*width*3+j*3];
			  ptr[3*j+1]=ptempImage[i*width*3+j*3+1];
			  ptr[3*j+2]=ptempImage[i*width*3+j*3+2];
	   }
	}

	//Inverting the rgb image data to bgr image data
	cv::cvtColor(cv_ptr_display->image ,cv_ptr_display->image, CV_RGB2BGR);

        //Displaying image
        cv::imshow(windowName, cv_ptr_display->image);
        cvWaitKey(30);

        //Freeing allocated memory
	free(ptempImage);
}
//*******************************************
//*********De codeBlocks a ROS***************
//*******************************************
//***************FIN PASO (5)****************
