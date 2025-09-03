package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDcontrolador {

  /*  Kp sempre > 0, pois e a potência do motor que vai ser aplicada
    Kp = define a força necessaria para se alcançar o erro ,mas causa oscilações
    Ki = corrige o erro acumulado com o tempo que Kp nao resolve
    Kd =  reduz oscilação e deixa mais estavel o controlador
            */
    double Kp, Ki, Kd;
    double ultimoerro = 0;
    double integral;
    double derivada;

     ElapsedTime timer = new ElapsedTime();
    public PIDcontrolador(double Kp, double Ki, double Kd) {

        this.Kp = Kp;// faz referencia as variaveis da funcao as igualando as variaveis da classe
        this.Ki = Ki;
        this.Kd = Kd;

    }

    public double calculo(double alvo, double atual) {
        // por favor funciona
        double erro = alvo - atual;

        double deltaTempo = timer.seconds();
        integral += erro * deltaTempo;
        derivada = (erro - ultimoerro) / deltaTempo;
        ultimoerro = erro;

        timer.reset();

        double saida = (Kp * erro) + (Ki * integral) + (Kd * derivada);


        return saida;
    }

    public void reset() {
        integral = 0;
        derivada = 0;
        ultimoerro = 0;
        timer.reset();
    }





}
