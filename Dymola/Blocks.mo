within ;
package Blocks
  "Library of basic input/output control blocks (continuous, discrete, logical, table blocks)"

  extends Modelica.Icons.Package;
  import Modelica.Units.SI;

package Examples
  "Library of examples to demonstrate the usage of package Blocks"

  extends Modelica.Icons.ExamplesPackage;

  model PID_Controller
    "Demonstrates the usage of a Continuous.LimPID controller"
    extends Modelica.Icons.Example;
    parameter SI.Angle driveAngle=1.570796326794897
      "Reference distance to move";
    Blocks.Continuous.LimPID PI(
        k=100,
        Ti=0.1,
        yMax=12,
        Ni=0.1,
        initType=Blocks.Types.Init.SteadyState,
        controllerType=Blocks.Types.SimpleController.PI,
        limiter(u(start=0)),
        Td=0.1)
        annotation (Placement(transformation(extent={{-56,-20},{-36,0}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia1(
      phi(fixed=true, start=0),
      J=1,
      a(fixed=true, start=0)) annotation (Placement(transformation(extent={{2,-20},
              {22,0}})));

    Modelica.Mechanics.Rotational.Sources.Torque torque annotation (Placement(
          transformation(extent={{-25,-20},{-5,0}})));
    Modelica.Mechanics.Rotational.Components.SpringDamper spring(
      c=1e4,
      d=100,
      stateSelect=StateSelect.prefer,
      w_rel(fixed=true)) annotation (Placement(transformation(extent={{32,-20},
              {52,0}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=2) annotation (
        Placement(transformation(extent={{60,-20},{80,0}})));
    Blocks.Sources.KinematicPTP kinematicPTP(
        startTime=0.5,
        deltaq={driveAngle},
        qd_max={1},
        qdd_max={1})
        annotation (Placement(transformation(extent={{-92,20},{-72,40}})));
    Blocks.Continuous.Integrator integrator(initType=Blocks.Types.Init.InitialState)
        annotation (Placement(transformation(extent={{-63,20},{-43,40}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
        Placement(transformation(extent={{22,-50},{2,-30}})));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque loadTorque(
        tau_constant=10, useSupport=false) annotation (Placement(transformation(
            extent={{98,-15},{88,-5}})));
  initial equation
    der(spring.w_rel) = 0;

  equation
    connect(spring.flange_b, inertia2.flange_a)
      annotation (Line(points={{52,-10},{60,-10}}));
    connect(inertia1.flange_b, spring.flange_a)
      annotation (Line(points={{22,-10},{32,-10}}));
    connect(torque.flange, inertia1.flange_a)
      annotation (Line(points={{-5,-10},{2,-10}}));
    connect(kinematicPTP.y[1], integrator.u)
      annotation (Line(points={{-71,30},{-65,30}}, color={0,0,127}));
    connect(speedSensor.flange, inertia1.flange_b)
      annotation (Line(points={{22,-40},{22,-10}}));
    connect(loadTorque.flange, inertia2.flange_b)
      annotation (Line(points={{88,-10},{80,-10}}));
    connect(PI.y, torque.tau)
      annotation (Line(points={{-35,-10},{-27,-10}}, color={0,0,127}));
    connect(speedSensor.w, PI.u_m)
      annotation (Line(points={{1,-40},{-46,-40},{-46,-22}}, color={0,0,127}));
    connect(integrator.y, PI.u_s) annotation (Line(points={{-42,30},{-37,30},{-37,
            11},{-67,11},{-67,-10},{-58,-10}}, color={0,0,127}));
    annotation (
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={
          Rectangle(extent={{-99,48},{-32,8}}, lineColor={255,0,0}),
          Text(
            extent={{-98,59},{-31,51}},
            textColor={255,0,0},
            textString="reference speed generation"),
          Text(
            extent={{-98,-46},{-60,-52}},
            textColor={255,0,0},
            textString="PI controller"),
          Line(
            points={{-76,-44},{-57,-23}},
            color={255,0,0},
            arrow={Arrow.None,Arrow.Filled}),
          Rectangle(extent={{-25,6},{99,-50}}, lineColor={255,0,0}),
          Text(
            extent={{4,14},{71,7}},
            textColor={255,0,0},
            textString="plant (simple drive train)")}),
      experiment(StopTime=4),
      Documentation(info="<html>

<p>
This is a simple drive train controlled by a PID controller:
</p>

<ul>
<li> The two blocks \"kinematic_PTP\" and \"integrator\" are used to generate
     the reference speed (= constant acceleration phase, constant speed phase,
     constant deceleration phase until inertia is at rest). To check
     whether the system starts in steady state, the reference speed is
     zero until time = 0.5 s and then follows the sketched trajectory.</li>

<li> The block \"PI\" is an instance of \"Blocks.Continuous.LimPID\" which is
     a PID controller where several practical important aspects, such as
     anti-windup-compensation has been added. In this case, the control block
     is used as PI controller.</li>

<li> The output of the controller is a torque that drives a motor inertia
     \"inertia1\". Via a compliant spring/damper component, the load
     inertia \"inertia2\" is attached. A constant external torque of 10 Nm
     is acting on the load inertia.</li>
</ul>

<p>
The PI controller is initialized in steady state (initType=SteadyState)
and the drive shall also be initialized in steady state.
However, it is not possible to initialize \"inertia1\" in SteadyState, because
\"der(inertia1.phi)=inertia1.w=0\" is an input to the PI controller that
defines that the derivative of the integrator state is zero (= the same
condition that was already defined by option SteadyState of the PI controller).
Furthermore, one initial condition is missing, because the absolute position
of inertia1 or inertia2 is not defined. The solution shown in this examples is
to initialize the angle and the angular acceleration of \"inertia1\".
</p>

<p>
In the following figure, results of a typical simulation are shown:
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/PID_controller.png\"
     alt=\"PID_controller.png\"><br>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/PID_controller2.png\"
     alt=\"PID_controller2.png\">

<p>
In the upper figure the reference speed (= integrator.y) and
the actual speed (= inertia1.w) are shown. As can be seen,
the system initializes in steady state, since no transients
are present. The inertia follows the reference speed quite good
until the end of the constant speed phase. Then there is a deviation.
In the lower figure the reason can be seen: The output of the
controller (PI.y) is in its limits. The anti-windup compensation
works reasonably, since the input to the limiter (PI.limiter.u)
is forced back to its limit after a transient phase.
</p>

</html>"));
  end PID_Controller;

  model Filter "Demonstrates the Continuous.Filter block with various options"
    extends Modelica.Icons.Example;
    parameter Integer order=3 "Number of order of filter";
    parameter SI.Frequency f_cut=2 "Cut-off frequency";
    parameter Blocks.Types.FilterType filterType=Blocks.Types.FilterType.LowPass
        "Type of filter (LowPass/HighPass)";
    parameter Blocks.Types.Init init=Blocks.Types.Init.SteadyState
        "Type of initialization (no init/steady state/initial state/initial output)";
    parameter Boolean normalized=true "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";

    Blocks.Sources.Step step(startTime=0.1, offset=0.1)
        annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
    Blocks.Continuous.Filter CriticalDamping(
        analogFilter=Blocks.Types.AnalogFilter.CriticalDamping,
        normalized=normalized,
        init=init,
        filterType=filterType,
        order=order,
        f_cut=f_cut,
        f_min=0.8*f_cut)
        annotation (Placement(transformation(extent={{-20,40},{0,60}})));
    Blocks.Continuous.Filter Bessel(
        normalized=normalized,
        analogFilter=Blocks.Types.AnalogFilter.Bessel,
        init=init,
        filterType=filterType,
        order=order,
        f_cut=f_cut,
        f_min=0.8*f_cut)
        annotation (Placement(transformation(extent={{-20,0},{0,20}})));
    Blocks.Continuous.Filter Butterworth(
        normalized=normalized,
        analogFilter=Blocks.Types.AnalogFilter.Butterworth,
        init=init,
        filterType=filterType,
        order=order,
        f_cut=f_cut,
        f_min=0.8*f_cut)
        annotation (Placement(transformation(extent={{-20,-40},{0,-20}})));
    Blocks.Continuous.Filter ChebyshevI(
        normalized=normalized,
        analogFilter=Blocks.Types.AnalogFilter.ChebyshevI,
        init=init,
        filterType=filterType,
        order=order,
        f_cut=f_cut,
        f_min=0.8*f_cut)
        annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));

  equation
    connect(step.y, CriticalDamping.u) annotation (Line(
        points={{-39,50},{-22,50}}, color={0,0,127}));
    connect(step.y, Bessel.u) annotation (Line(
        points={{-39,50},{-32,50},{-32,10},{-22,10}}, color={0,0,127}));
    connect(Butterworth.u, step.y) annotation (Line(
        points={{-22,-30},{-32,-30},{-32,50},{-39,50}}, color={0,0,127}));
    connect(ChebyshevI.u, step.y) annotation (Line(
        points={{-22,-70},{-32,-70},{-32,50},{-39,50}}, color={0,0,127}));
    annotation (
      experiment(StopTime=0.9),
      Documentation(info="<html>

<p>
This example demonstrates various options of the
<a href=\"modelica://Modelica.Blocks.Continuous.Filter\">Filter</a> block.
A step input starts at 0.1 s with an offset of 0.1, in order to demonstrate
the initialization options. This step input drives 4 filter blocks that
have identical parameters, with the only exception of the used analog filter type
(CriticalDamping, Bessel, Butterworth, Chebyshev of type I). All the main options
can be set via parameters and are then applied to all the 4 filters.
The default setting uses low pass filters of order 3 with a cut-off frequency of
2 Hz resulting in the following outputs:
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Filter1.png\"
     alt=\"Filter1.png\">
</html>"));
  end Filter;

  model FilterWithDifferentiation
    "Demonstrates the use of low pass filters to determine derivatives of filters"
    extends Modelica.Icons.Example;
    parameter SI.Frequency f_cut=2 "Cut-off frequency";

    Blocks.Sources.Step step(startTime=0.1, offset=0.1)
        annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
    Blocks.Continuous.Filter Bessel(
        f_cut=f_cut,
        filterType=Blocks.Types.FilterType.LowPass,
        order=3,
        analogFilter=Blocks.Types.AnalogFilter.Bessel)
        annotation (Placement(transformation(extent={{-40,40},{-20,60}})));

    Continuous.Der der1
      annotation (Placement(transformation(extent={{-6,40},{14,60}})));
    Continuous.Der der2
      annotation (Placement(transformation(extent={{30,40},{50,60}})));
    Continuous.Der der3
      annotation (Placement(transformation(extent={{62,40},{82,60}})));
  equation
    connect(step.y, Bessel.u) annotation (Line(
        points={{-59,50},{-42,50}}, color={0,0,127}));
    connect(Bessel.y, der1.u) annotation (Line(
        points={{-19,50},{-8,50}}, color={0,0,127}));
    connect(der1.y, der2.u) annotation (Line(
        points={{15,50},{28,50}}, color={0,0,127}));
    connect(der2.y, der3.u) annotation (Line(
        points={{51,50},{60,50}}, color={0,0,127}));
    annotation (
      experiment(StopTime=0.9),
      Documentation(info="<html>

<p>
This example demonstrates that the output of the
<a href=\"modelica://Modelica.Blocks.Continuous.Filter\">Filter</a> block
can be differentiated up to the order of the filter. This feature can be
used in order to make an inverse model realizable or to \"smooth\" a potential
discontinuous control signal.
</p>

</html>"));
  end FilterWithDifferentiation;

  model FilterWithRiseTime
    "Demonstrates to use the rise time instead of the cut-off frequency to define a filter"
    import Modelica.Constants.pi;
    extends Modelica.Icons.Example;
    parameter Integer order=2 "Filter order";
    parameter SI.Time riseTime=2 "Time to reach the step input";

    Continuous.Filter filter_fac5(f_cut=5/(2*pi*riseTime), order=order)
      annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
    Sources.Step step(startTime=1)
      annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
    Continuous.Filter filter_fac4(f_cut=4/(2*pi*riseTime), order=order)
      annotation (Placement(transformation(extent={{-20,20},{0,40}})));
    Continuous.Filter filter_fac3(f_cut=3/(2*pi*riseTime), order=order)
      annotation (Placement(transformation(extent={{-20,62},{0,82}})));
  equation
    connect(step.y, filter_fac5.u) annotation (Line(
        points={{-39,30},{-30,30},{-30,-10},{-22,-10}}, color={0,0,127}));
    connect(step.y, filter_fac4.u) annotation (Line(
        points={{-39,30},{-22,30}}, color={0,0,127}));
    connect(step.y, filter_fac3.u) annotation (Line(
        points={{-39,30},{-30,30},{-30,72},{-22,72}}, color={0,0,127}));
    annotation (experiment(StopTime=4), Documentation(info="<html>
<p>
Filters are usually parameterized with the cut-off frequency.
Sometimes, it is more meaningful to parameterize a filter with its
rise time, i.e., the time it needs until the output reaches the end value
of a step input. This is performed with the formula:
</p>

<blockquote><pre>
f_cut = fac/(2*pi*riseTime);
</pre></blockquote>

<p>
where \"fac\" is typically 3, 4, or 5. The following image shows
the results of a simulation of this example model
(riseTime = 2 s, fac=3, 4, and 5):
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/FilterWithRiseTime.png\"
     alt=\"FilterWithRiseTime.png\">

<p>
Since the step starts at 1 s, and the rise time is 2 s, the filter output y
shall reach the value of 1 after 1+2=3 s. Depending on the factor \"fac\" this is
reached with different precisions. This is summarized in the following table:
</p>

<blockquote><table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr>
   <td>Filter order</td>
   <td>Factor fac</td>
   <td>Percentage of step value reached after rise time</td>
</tr>
<tr>
   <td align=\"center\">1</td>
   <td align=\"center\">3</td>
   <td align=\"center\">95.1 %</td>
</tr>
<tr>
   <td align=\"center\">1</td>
   <td align=\"center\">4</td>
   <td align=\"center\">98.2 %</td>
</tr>
<tr>
   <td align=\"center\">1</td>
   <td align=\"center\">5</td>
   <td align=\"center\">99.3 %</td>
</tr>

<tr>
   <td align=\"center\">2</td>
   <td align=\"center\">3</td>
   <td align=\"center\">94.7 %</td>
</tr>
<tr>
   <td align=\"center\">2</td>
   <td align=\"center\">4</td>
   <td align=\"center\">98.6 %</td>
</tr>
<tr>
   <td align=\"center\">2</td>
   <td align=\"center\">5</td>
   <td align=\"center\">99.6 %</td>
</tr>
</table></blockquote>

</html>"));
  end FilterWithRiseTime;

  model SlewRateLimiter
    "Demonstrate usage of Nonlinear.SlewRateLimiter"
    extends Modelica.Icons.Example;
    parameter SI.Velocity vMax=2 "Max. velocity";
    parameter SI.Acceleration aMax=20 "Max. acceleration";
    SI.Position s=positionStep.y "Reference position";
    SI.Position sSmoothed=positionSmoothed.y "Smoothed position";
    SI.Velocity vLimited=limit_a.y "Limited velocity";
    SI.Acceleration aLimited=a.y "Limited acceleration";
    Blocks.Sources.Step positionStep(startTime=0.1)
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Blocks.Nonlinear.SlewRateLimiter limit_v(
        initType=Blocks.Types.Init.InitialOutput,
        Rising=vMax,
        y_start=positionStep.offset,
        Td=0.0001)
        annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
    Blocks.Continuous.Der v
        annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
    Blocks.Nonlinear.SlewRateLimiter limit_a(
        initType=Blocks.Types.Init.InitialOutput,
        y_start=0,
        Rising=20,
        Td=0.0001)
        annotation (Placement(transformation(extent={{10,-10},{30,10}})));
    Blocks.Continuous.Integrator positionSmoothed(
        k=1,
        initType=Blocks.Types.Init.InitialOutput,
        y_start=positionStep.offset)
        annotation (Placement(transformation(extent={{50,-10},{70,10}})));
    Blocks.Continuous.Der a
        annotation (Placement(transformation(extent={{50,-40},{70,-20}})));
  equation
    connect(positionStep.y, limit_v.u)
      annotation (Line(points={{-59,0},{-52,0}}, color={0,0,127}));
    connect(limit_v.y, v.u)
      annotation (Line(points={{-29,0},{-22,0}}, color={0,0,127}));
    connect(v.y, limit_a.u)
      annotation (Line(points={{1,0},{8,0}}, color={0,0,127}));
    connect(limit_a.y, positionSmoothed.u)
      annotation (Line(points={{31,0},{39.5,0},{48,0}}, color={0,0,127}));
    connect(limit_a.y, a.u) annotation (Line(points={{31,0},{40,0},{40,-30},{48,-30}},
          color={0,0,127}));

    annotation (experiment(StopTime=1.0, Interval=0.001), Documentation(info="<html>
<p>
This example demonstrates how to use the Nonlinear.SlewRateLimiter block to limit a position step with regards to velocity and acceleration:
</p>
<ul>
<li> The Sources.Step block <code>positionStep</code> demands an unphysical position step.</li>
<li> The first SlewRateLimiter block  <code>limit_v</code> limits velocity.</li>
<li> The first Der block <code>v</code> calculates velocity from the smoothed position signal.</li>
<li> The second SlewRateLimiter block <code>limit_a</code> limits acceleration of the smoothed velocity signal.</li>
<li> The second Der block <code>a</code> calculates acceleration from the smoothed velocity signal.</li>
<li> The Integrator block <code>positionSmoothed</code> calculates smoothed position from the smoothed velocity signal.</li>
</ul>
<p>
A position controlled drive with limited velocity and limited acceleration (i.e. torque) is able to follow the smoothed reference position.
</p>
</html>"));
  end SlewRateLimiter;

  model InverseModel "Demonstrates the construction of an inverse model"
    extends Modelica.Icons.Example;

    Continuous.FirstOrder firstOrder1(
      k=1,
      T=0.3,
      initType=Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(extent={{20,20},{0,40}})));
    Sources.Sine sine(
      f=2,
      offset=1,
      startTime=0.2)
      annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
    Math.InverseBlockConstraints inverseBlockConstraints
      annotation (Placement(transformation(extent={{-10,20},{30,40}})));
    Continuous.FirstOrder firstOrder2(
      k=1,
      T=0.3,
      initType=Blocks.Types.Init.SteadyState)
      annotation (Placement(transformation(extent={{20,-20},{0,0}})));
    Math.Feedback feedback
      annotation (Placement(transformation(extent={{-40,0},{-60,-20}})));
    Continuous.CriticalDamping criticalDamping(n=1, f=50*sine.f)
      annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  equation
    connect(firstOrder1.y, inverseBlockConstraints.u2) annotation (Line(
        points={{-1,30},{-6,30}}, color={0,0,127}));
    connect(inverseBlockConstraints.y2, firstOrder1.u) annotation (Line(
        points={{27,30},{22,30}}, color={0,0,127}));
    connect(firstOrder2.y, feedback.u1) annotation (Line(
        points={{-1,-10},{-42,-10}}, color={0,0,127}));
    connect(sine.y, criticalDamping.u) annotation (Line(
        points={{-59,30},{-42,30}}, color={0,0,127}));
    connect(criticalDamping.y, inverseBlockConstraints.u1) annotation (Line(
        points={{-19,30},{-12,30}}, color={0,0,127}));
    connect(sine.y, feedback.u2) annotation (Line(
        points={{-59,30},{-50,30},{-50,-2}}, color={0,0,127}));
    connect(inverseBlockConstraints.y1, firstOrder2.u) annotation (Line(
        points={{31,30},{40,30},{40,-10},{22,-10}}, color={0,0,127}));
    annotation (Documentation(info="<html>
<p>
This example demonstrates how to construct an inverse model in Modelica
(for more details see <a href=\"https://www.modelica.org/events/Conference2005/online_proceedings/Session3/Session3c3.pdf\">Looye, Th&uuml;mmel, Kurze, Otter, Bals: Nonlinear Inverse Models for Control</a>).
</p>

<p>
For a linear, single input, single output system
</p>

<blockquote><pre>
y = n(s)/d(s) * u   // plant model
</pre></blockquote>

<p>
the inverse model is derived by simply exchanging the numerator and
the denominator polynomial:
</p>

<blockquote><pre>
u = d(s)/n(s) * y   // inverse plant model
</pre></blockquote>

<p>
If the denominator polynomial d(s) has a higher degree as the
numerator polynomial n(s) (which is usually the case for plant models),
then the inverse model is no longer proper, i.e., it is not causal.
To avoid this, an approximate inverse model is constructed by adding
a sufficient number of poles to the denominator of the inverse model.
This can be interpreted as filtering the desired output signal y:
</p>

<blockquote><pre>
u = d(s)/(n(s)*f(s)) * y  // inverse plant model with filtered y
</pre></blockquote>

<p>
With Modelica it is in principal possible to construct inverse models not only
for linear but also for non-linear models and in particular for every
Modelica model. The basic construction mechanism is explained at hand
of this example:
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/InverseModelSchematic.png\"
     alt=\"InverseModelSchematic.png\">

<p>
Here the first order block \"firstOrder1\" shall be inverted. This is performed
by connecting its inputs and outputs with an instance of block
Modelica.Blocks.Math.<strong>InverseBlockConstraints</strong>. By this connection,
the inputs and outputs are exchanged. The goal is to compute the input of the
\"firstOrder1\" block so that its output follows a given sine signal.
For this, the sine signal \"sin\" is first filtered with a \"CriticalDamping\"
filter of order 1 and then the output of this filter is connected to the output
of the \"firstOrder1\" block (via the InverseBlockConstraints block, since
2 outputs cannot be connected directly together in a block diagram).
</p>

<p>
In order to check the inversion, the computed input of \"firstOrder1\" is used
as input in an identical block \"firstOrder2\". The output of \"firstOrder2\" should
be the given \"sine\" function. The difference is constructed with the \"feedback\"
block. Since the \"sine\" function is filtered, one cannot expect that this difference
is zero. The higher the cut-off frequency of the filter, the closer is the
agreement. A typical simulation result is shown in the next figure:
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/InverseModel.png\"
     alt=\"InverseModel.png\">
</html>"), experiment(StopTime=1.0));
  end InverseModel;

  model ShowLogicalSources
    "Demonstrates the usage of logical sources together with their diagram animation"
    extends Modelica.Icons.Example;
    Sources.BooleanTable table(table={2,4,6,8}) annotation (Placement(
          transformation(extent={{-60,-100},{-40,-80}})));
    Sources.BooleanConstant const annotation (Placement(transformation(extent={
              {-60,60},{-40,80}})));
    Sources.BooleanStep step(startTime=4) annotation (Placement(transformation(
            extent={{-60,20},{-40,40}})));
    Sources.BooleanPulse pulse(period=1.5) annotation (Placement(transformation(
            extent={{-60,-20},{-40,0}})));

    Sources.SampleTrigger sample(period=0.5) annotation (Placement(
          transformation(extent={{-60,-60},{-40,-40}})));
    Sources.BooleanExpression booleanExpression(y=pulse.y and step.y)
      annotation (Placement(transformation(extent={{20,20},{80,40}})));
    annotation (experiment(StopTime=10), Documentation(info="<html>
<p>
This simple example demonstrates the logical sources in
<a href=\"modelica://Modelica.Blocks.Sources\">Modelica.Blocks.Sources</a> and demonstrate
their diagram animation (see \"small circle\" close to the output connector).
The \"booleanExpression\" source shows how a logical expression can be defined
in its parameter menu referring to variables available on this level of the
model.
</p>

</html>"));
  end ShowLogicalSources;

  model LogicalNetwork1 "Demonstrates the usage of logical blocks"

    extends Modelica.Icons.Example;
    Sources.BooleanTable table2(table={1,3,5,7}) annotation (Placement(
          transformation(extent={{-80,-20},{-60,0}})));
    Sources.BooleanTable table1(table={2,4,6,8}) annotation (Placement(
          transformation(extent={{-80,20},{-60,40}})));
    Logical.Not Not1 annotation (Placement(transformation(extent={{-40,-20},{-20,
              0}})));

    Logical.And And1 annotation (Placement(transformation(extent={{0,-20},{20,0}})));
    Logical.Or Or1 annotation (Placement(transformation(extent={{40,20},{60,40}})));
    Logical.Pre Pre1 annotation (Placement(transformation(extent={{-40,-60},{-20,
              -40}})));
  equation

    connect(table2.y, Not1.u)
      annotation (Line(points={{-59,-10},{-42,-10}}, color={255,0,255}));
    connect(And1.y, Or1.u2) annotation (Line(points={{21,-10},{28,-10},{28,22},
            {38,22}}, color={255,0,255}));
    connect(table1.y, Or1.u1)
      annotation (Line(points={{-59,30},{38,30}}, color={255,0,255}));
    connect(Not1.y, And1.u1)
      annotation (Line(points={{-19,-10},{-2,-10}}, color={255,0,255}));
    connect(Pre1.y, And1.u2) annotation (Line(points={{-19,-50},{-10,-50},{-10,
            -18},{-2,-18}}, color={255,0,255}));
    connect(Or1.y, Pre1.u) annotation (Line(points={{61,30},{68,30},{68,-70},{-60,
            -70},{-60,-50},{-42,-50}}, color={255,0,255}));

    annotation (experiment(StopTime=10), Documentation(info="<html>
<p>
This example demonstrates a network of logical blocks. Note, that
the Boolean values of the input and output signals are visualized
in the diagram animation, by the small \"circles\" close to the connectors.
If a \"circle\" is \"white\", the signal is <strong>false</strong>. It a
\"circle\" is \"green\", the signal is <strong>true</strong>.
</p>
</html>"));
  end LogicalNetwork1;

model RealNetwork1 "Demonstrates the usage of blocks from Modelica.Blocks.Math"

  extends Modelica.Icons.Example;

  Blocks.Math.MultiSum add(nu=2)
        annotation (Placement(transformation(extent={{-14,64},{-2,76}})));
  Sources.Sine sine(amplitude=3, f=0.1)
    annotation (Placement(transformation(extent={{-96,60},{-76,80}})));
  Sources.Step integerStep(height=3, startTime=2)
    annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
  Sources.Constant integerConstant(k=1)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Blocks.Interaction.Show.RealValue showValue
        annotation (Placement(transformation(extent={{66,60},{86,80}})));
  Math.MultiProduct product(nu=2)
    annotation (Placement(transformation(extent={{6,24},{18,36}})));
  Blocks.Interaction.Show.RealValue showValue1(significantDigits=2)
        annotation (Placement(transformation(extent={{64,20},{84,40}})));
  Sources.BooleanPulse booleanPulse1(period=1)
    annotation (Placement(transformation(extent={{-12,-30},{8,-10}})));
  Math.MultiSwitch multiSwitch(
    nu=2,
    expr={4,6},
    y_default=2)
    annotation (Placement(transformation(extent={{28,-60},{68,-40}})));
  Sources.BooleanPulse booleanPulse2(period=2, width=80)
    annotation (Placement(transformation(extent={{-12,-70},{8,-50}})));
  Blocks.Interaction.Show.RealValue showValue3(
        use_numberPort=false,
        number=multiSwitch.y,
        significantDigits=1)
        annotation (Placement(transformation(extent={{40,-84},{60,-64}})));
  Math.LinearDependency linearDependency1(
    y0=1,
    k1=2,
    k2=3) annotation (Placement(transformation(extent={{40,80},{60,100}})));
  Math.MinMax minMax(nu=2)
    annotation (Placement(transformation(extent={{58,-16},{78,4}})));
equation
  connect(booleanPulse1.y, multiSwitch.u[1]) annotation (Line(
      points={{9,-20},{18,-20},{18,-48},{28,-48},{28,-48.5}}, color={255,0,255}));
  connect(booleanPulse2.y, multiSwitch.u[2]) annotation (Line(
      points={{9,-60},{18,-60},{18,-52},{28,-52},{28,-51.5}}, color={255,0,255}));
  connect(sine.y, add.u[1]) annotation (Line(
      points={{-75,70},{-46.5,70},{-46.5,72.1},{-14,72.1}}, color={0,0,127}));
  connect(integerStep.y, add.u[2]) annotation (Line(
      points={{-39,40},{-28,40},{-28,67.9},{-14,67.9}}, color={0,0,127}));
  connect(add.y, showValue.numberPort) annotation (Line(
      points={{-0.98,70},{64.5,70}}, color={0,0,127}));
  connect(integerStep.y, product.u[1]) annotation (Line(
      points={{-39,40},{-20,40},{-20,32.1},{6,32.1}}, color={0,0,127}));
  connect(integerConstant.y, product.u[2]) annotation (Line(
      points={{-39,0},{-20,0},{-20,27.9},{6,27.9}}, color={0,0,127}));
  connect(product.y, showValue1.numberPort) annotation (Line(
      points={{19.02,30},{62.5,30}}, color={0,0,127}));
  connect(add.y, linearDependency1.u1) annotation (Line(
      points={{-0.98,70},{20,70},{20,96},{38,96}}, color={0,0,127}));
  connect(product.y, linearDependency1.u2) annotation (Line(
      points={{19.02,30},{30,30},{30,84},{38,84}}, color={0,0,127}));
  connect(add.y, minMax.u[1]) annotation (Line(
      points={{-0.98,70},{48,70},{48,-2.5},{58,-2.5}}, color={0,0,127}));
  connect(product.y, minMax.u[2]) annotation (Line(
      points={{19.02,30},{40,30},{40,-9.5},{58,-9.5}}, color={0,0,127}));
  annotation (
    experiment(StopTime=10),
    Documentation(info="<html>
<p>
This example demonstrates a network of mathematical Real blocks.
from package <a href=\"modelica://Modelica.Blocks.Math\">Modelica.Blocks.Math</a>.
Note, that
</p>

<ul>
<li> at the right side of the model, several Math.ShowValue blocks
     are present, that visualize the actual value of the respective Real
     signal in a diagram animation.</li>

<li> the Boolean values of the input and output signals are visualized
     in the diagram animation, by the small \"circles\" close to the connectors.
     If a \"circle\" is \"white\", the signal is <strong>false</strong>. If a
     \"circle\" is \"green\", the signal is <strong>true</strong>.</li>
</ul>

</html>"));
end RealNetwork1;

  model IntegerNetwork1
    "Demonstrates the usage of blocks from Modelica.Blocks.MathInteger"

    extends Modelica.Icons.Example;

    MathInteger.Sum sum(nu=3)
      annotation (Placement(transformation(extent={{-14,64},{-2,76}})));
    Sources.Sine sine(amplitude=3, f=0.1)
      annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
    Math.RealToInteger realToInteger
      annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
    Sources.IntegerStep integerStep(height=3, startTime=2)
      annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
    Sources.IntegerConstant integerConstant(k=1)
      annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
    Blocks.Interaction.Show.IntegerValue showValue
        annotation (Placement(transformation(extent={{40,60},{60,80}})));
    MathInteger.Product product(nu=2)
      annotation (Placement(transformation(extent={{16,24},{28,36}})));
    Blocks.Interaction.Show.IntegerValue showValue1
        annotation (Placement(transformation(extent={{40,20},{60,40}})));
    MathInteger.TriggeredAdd triggeredAdd(use_reset=false, use_set=false)
      annotation (Placement(transformation(extent={{16,-6},{28,6}})));
    Sources.BooleanPulse booleanPulse1(period=1)
      annotation (Placement(transformation(extent={{-12,-30},{8,-10}})));
    Blocks.Interaction.Show.IntegerValue showValue2
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    MathInteger.MultiSwitch multiSwitch1(
      nu=2,
      expr={4,6},
      y_default=2,
      use_pre_as_default=false)
      annotation (Placement(transformation(extent={{28,-60},{68,-40}})));
    Sources.BooleanPulse booleanPulse2(period=2, width=80)
      annotation (Placement(transformation(extent={{-12,-70},{8,-50}})));
    Blocks.Interaction.Show.IntegerValue showValue3(use_numberPort=false,
          number=multiSwitch1.y)
        annotation (Placement(transformation(extent={{40,-84},{60,-64}})));
  equation
    connect(sine.y, realToInteger.u) annotation (Line(
        points={{-79,70},{-62,70}}, color={0,0,127}));
    connect(realToInteger.y, sum.u[1]) annotation (Line(
        points={{-39,70},{-32,70},{-32,72},{-14,72},{-14,72.8}}, color={255,127,0}));
    connect(integerStep.y, sum.u[2]) annotation (Line(
        points={{-39,40},{-28,40},{-28,70},{-14,70}}, color={255,127,0}));
    connect(integerConstant.y, sum.u[3]) annotation (Line(
        points={{-39,0},{-22,0},{-22,67.2},{-14,67.2}}, color={255,127,0}));
    connect(sum.y, showValue.numberPort) annotation (Line(
        points={{-1.1,70},{38.5,70}}, color={255,127,0}));
    connect(sum.y, product.u[1]) annotation (Line(
        points={{-1.1,70},{4,70},{4,32.1},{16,32.1}}, color={255,127,0}));
    connect(integerStep.y, product.u[2]) annotation (Line(
        points={{-39,40},{-8,40},{-8,27.9},{16,27.9}}, color={255,127,0}));
    connect(product.y, showValue1.numberPort) annotation (Line(
        points={{28.9,30},{38.5,30}}, color={255,127,0}));
    connect(integerConstant.y, triggeredAdd.u) annotation (Line(
        points={{-39,0},{13.6,0}}, color={255,127,0}));
    connect(booleanPulse1.y, triggeredAdd.trigger) annotation (Line(
        points={{9,-20},{18.4,-20},{18.4,-7.2}}, color={255,0,255}));
    connect(triggeredAdd.y, showValue2.numberPort) annotation (Line(
        points={{29.2,0},{38.5,0}}, color={255,127,0}));
    connect(booleanPulse1.y, multiSwitch1.u[1]) annotation (Line(
        points={{9,-20},{18,-20},{18,-48},{28,-48},{28,-48.5}}, color={255,0,255}));
    connect(booleanPulse2.y, multiSwitch1.u[2]) annotation (Line(
        points={{9,-60},{18,-60},{18,-52},{28,-52},{28,-51.5}}, color={255,0,255}));
    annotation (experiment(StopTime=10), Documentation(info="<html>
<p>
This example demonstrates a network of Integer blocks.
from package <a href=\"modelica://Modelica.Blocks.MathInteger\">Modelica.Blocks.MathInteger</a>.
Note, that
</p>

<ul>
<li> at the right side of the model, several MathInteger.ShowValue blocks
     are present, that visualize the actual value of the respective Integer
     signal in a diagram animation.</li>

<li> the Boolean values of the input and output signals are visualized
     in the diagram animation, by the small \"circles\" close to the connectors.
     If a \"circle\" is \"white\", the signal is <strong>false</strong>. If a
     \"circle\" is \"green\", the signal is <strong>true</strong>.</li>

</ul>

</html>"));
  end IntegerNetwork1;

  model BooleanNetwork1
    "Demonstrates the usage of blocks from Modelica.Blocks.MathBoolean"

    extends Modelica.Icons.Example;

    Blocks.Interaction.Show.BooleanValue showValue
        annotation (Placement(transformation(extent={{-36,74},{-16,94}})));
    MathBoolean.And and1(nu=3)
      annotation (Placement(transformation(extent={{-58,64},{-46,76}})));
    Sources.BooleanPulse booleanPulse1(width=20, period=1)
      annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
    Sources.BooleanPulse booleanPulse2(period=1, width=80)
      annotation (Placement(transformation(extent={{-100,-4},{-80,16}})));
    Sources.BooleanStep booleanStep(startTime=1.5)
      annotation (Placement(transformation(extent={{-100,28},{-80,48}})));
    MathBoolean.Or or1(nu=2)
      annotation (Placement(transformation(extent={{-28,62},{-16,74}})));
    MathBoolean.Xor xor1(nu=2)
      annotation (Placement(transformation(extent={{-2,60},{10,72}})));
    Blocks.Interaction.Show.BooleanValue showValue2
        annotation (Placement(transformation(extent={{-2,74},{18,94}})));
    Blocks.Interaction.Show.BooleanValue showValue3
        annotation (Placement(transformation(extent={{24,56},{44,76}})));
    MathBoolean.Nand nand1(nu=2)
      annotation (Placement(transformation(extent={{22,40},{34,52}})));
    MathBoolean.Nor or2(nu=2)
      annotation (Placement(transformation(extent={{46,38},{58,50}})));
    Blocks.Interaction.Show.BooleanValue showValue4
        annotation (Placement(transformation(extent={{90,34},{110,54}})));
    MathBoolean.Not nor1
      annotation (Placement(transformation(extent={{68,40},{76,48}})));
    MathBoolean.OnDelay onDelay(delayTime=1)
      annotation (Placement(transformation(extent={{-56,-94},{-48,-86}})));
    MathBoolean.RisingEdge rising
      annotation (Placement(transformation(extent={{-56,-15},{-48,-7}})));
    MathBoolean.MultiSwitch set1(nu=2, expr={false,true})
      annotation (Placement(transformation(extent={{-30,-23},{10,-3}})));
    MathBoolean.FallingEdge falling
      annotation (Placement(transformation(extent={{-56,-32},{-48,-24}})));
    Sources.BooleanTable booleanTable(table={2,4,6,6.5,7,9,11})
      annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
    MathBoolean.ChangingEdge changing
      annotation (Placement(transformation(extent={{-56,-59},{-48,-51}})));
    MathInteger.TriggeredAdd triggeredAdd
      annotation (Placement(transformation(extent={{14,-56},{26,-44}})));
    Sources.IntegerConstant integerConstant(k=2)
      annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
    Blocks.Interaction.Show.IntegerValue showValue1
        annotation (Placement(transformation(extent={{40,-60},{60,-40}})));
    Blocks.Interaction.Show.BooleanValue showValue5
        annotation (Placement(transformation(extent={{24,-23},{44,-3}})));
    Blocks.Interaction.Show.BooleanValue showValue6
        annotation (Placement(transformation(extent={{-32,-100},{-12,-80}})));
    Logical.RSFlipFlop rSFlipFlop
      annotation (Placement(transformation(extent={{70,-90},{90,-70}})));
    Sources.SampleTrigger sampleTriggerSet(period=0.5, startTime=0)
      annotation (Placement(transformation(extent={{40,-76},{54,-62}})));
    Sources.SampleTrigger sampleTriggerReset(period=0.5, startTime=0.3)
      annotation (Placement(transformation(extent={{40,-98},{54,-84}})));
  equation
    connect(booleanPulse1.y, and1.u[1]) annotation (Line(
        points={{-79,70},{-68,70},{-68,72.8},{-58,72.8}}, color={255,0,255}));
    connect(booleanStep.y, and1.u[2]) annotation (Line(
        points={{-79,38},{-64,38},{-64,70},{-58,70}}, color={255,0,255}));
    connect(booleanPulse2.y, and1.u[3]) annotation (Line(
        points={{-79,6},{-62,6},{-62,67.2},{-58,67.2}}, color={255,0,255}));
    connect(and1.y, or1.u[1]) annotation (Line(
        points={{-45.1,70},{-36.4,70},{-36.4,70.1},{-28,70.1}}, color={255,0,255}));
    connect(booleanPulse2.y, or1.u[2]) annotation (Line(
        points={{-79,6},{-40,6},{-40,65.9},{-28,65.9}}, color={255,0,255}));
    connect(or1.y, xor1.u[1]) annotation (Line(
        points={{-15.1,68},{-8,68},{-8,68.1},{-2,68.1}}, color={255,0,255}));
    connect(booleanPulse2.y, xor1.u[2]) annotation (Line(
        points={{-79,6},{-12,6},{-12,63.9},{-2,63.9}}, color={255,0,255}));
    connect(and1.y, showValue.activePort) annotation (Line(
        points={{-45.1,70},{-42,70},{-42,84},{-37.5,84}}, color={255,0,255}));
    connect(or1.y, showValue2.activePort) annotation (Line(
        points={{-15.1,68},{-12,68},{-12,84},{-3.5,84}}, color={255,0,255}));
    connect(xor1.y, showValue3.activePort) annotation (Line(
        points={{10.9,66},{22.5,66}}, color={255,0,255}));
    connect(xor1.y, nand1.u[1]) annotation (Line(
        points={{10.9,66},{16,66},{16,48.1},{22,48.1}}, color={255,0,255}));
    connect(booleanPulse2.y, nand1.u[2]) annotation (Line(
        points={{-79,6},{16,6},{16,44},{22,44},{22,43.9}}, color={255,0,255}));
    connect(nand1.y, or2.u[1]) annotation (Line(
        points={{34.9,46},{46,46},{46,46.1}}, color={255,0,255}));
    connect(booleanPulse2.y, or2.u[2]) annotation (Line(
        points={{-79,6},{42,6},{42,41.9},{46,41.9}}, color={255,0,255}));
    connect(or2.y, nor1.u) annotation (Line(
        points={{58.9,44},{66.4,44}}, color={255,0,255}));
    connect(nor1.y, showValue4.activePort) annotation (Line(
        points={{76.8,44},{88.5,44}}, color={255,0,255}));
    connect(booleanPulse2.y, rising.u) annotation (Line(
        points={{-79,6},{-62,6},{-62,-11},{-57.6,-11}}, color={255,0,255}));
    connect(rising.y, set1.u[1]) annotation (Line(
        points={{-47.2,-11},{-38.6,-11},{-38.6,-11.5},{-30,-11.5}}, color={255,0,255}));
    connect(falling.y, set1.u[2]) annotation (Line(
        points={{-47.2,-28},{-40,-28},{-40,-14.5},{-30,-14.5}}, color={255,0,255}));
    connect(booleanPulse2.y, falling.u) annotation (Line(
        points={{-79,6},{-62,6},{-62,-28},{-57.6,-28}}, color={255,0,255}));
    connect(booleanTable.y, onDelay.u) annotation (Line(
        points={{-79,-90},{-57.6,-90}}, color={255,0,255}));
    connect(booleanPulse2.y, changing.u) annotation (Line(
        points={{-79,6},{-62,6},{-62,-55},{-57.6,-55}}, color={255,0,255}));
    connect(integerConstant.y, triggeredAdd.u) annotation (Line(
        points={{1,-50},{11.6,-50}}, color={255,127,0}));
    connect(changing.y, triggeredAdd.trigger) annotation (Line(
        points={{-47.2,-55},{-30,-55},{-30,-74},{16.4,-74},{16.4,-57.2}}, color={255,0,255}));
    connect(triggeredAdd.y, showValue1.numberPort) annotation (Line(
        points={{27.2,-50},{38.5,-50}}, color={255,127,0}));
    connect(set1.y, showValue5.activePort) annotation (Line(
        points={{11,-13},{22.5,-13}}, color={255,0,255}));
    connect(onDelay.y, showValue6.activePort) annotation (Line(
        points={{-47.2,-90},{-33.5,-90}}, color={255,0,255}));
    connect(sampleTriggerSet.y, rSFlipFlop.S) annotation (Line(
        points={{54.7,-69},{60,-69},{60,-74},{68,-74}}, color={255,0,255}));
    connect(sampleTriggerReset.y, rSFlipFlop.R) annotation (Line(
        points={{54.7,-91},{60,-91},{60,-86},{68,-86}}, color={255,0,255}));
    annotation (experiment(StopTime=10), Documentation(info="<html>
<p>
This example demonstrates a network of Boolean blocks
from package <a href=\"modelica://Modelica.Blocks.MathBoolean\">Modelica.Blocks.MathBoolean</a>.
Note, that
</p>

<ul>
<li> at the right side of the model, several MathBoolean.ShowValue blocks
     are present, that visualize the actual value of the respective Boolean
     signal in a diagram animation (\"green\" means \"true\").</li>

<li> the Boolean values of the input and output signals are visualized
     in the diagram animation, by the small \"circles\" close to the connectors.
     If a \"circle\" is \"white\", the signal is <strong>false</strong>. If a
     \"circle\" is \"green\", the signal is <strong>true</strong>.</li>

</ul>

</html>"));
  end BooleanNetwork1;

  model Interaction1
    "Demonstrates the usage of blocks from Modelica.Blocks.Interaction.Show"

    extends Modelica.Icons.Example;

    Interaction.Show.IntegerValue integerValue
      annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
    Sources.IntegerTable integerTable(table=[0, 0; 1, 2; 2, 4; 3, 6; 4, 4; 6, 2])
      annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
    Sources.TimeTable timeTable(table=[0, 0; 1, 2.1; 2, 4.2; 3, 6.3; 4, 4.2; 6,
          2.1; 6, 2.1])
      annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
    Interaction.Show.RealValue realValue
      annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
    Sources.BooleanTable booleanTable(table={1,2,3,4,5,6,7,8,9})
      annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
    Interaction.Show.BooleanValue booleanValue
      annotation (Placement(transformation(extent={{-40,-20},{-20,0}})));
    Sources.RadioButtonSource start(buttonTimeTable={1,3}, reset={stop.on})
      annotation (Placement(transformation(extent={{24,64},{36,76}})));
    Sources.RadioButtonSource stop(buttonTimeTable={2,4}, reset={start.on})
      annotation (Placement(transformation(extent={{48,64},{60,76}})));
  equation
    connect(integerTable.y, integerValue.numberPort) annotation (Line(
        points={{-59,30},{-41.5,30}}, color={255,127,0}));
    connect(timeTable.y, realValue.numberPort) annotation (Line(
        points={{-59,70},{-41.5,70}}, color={0,0,127}));
    connect(booleanTable.y, booleanValue.activePort) annotation (Line(
        points={{-59,-10},{-41.5,-10}}, color={255,0,255}));
    annotation (experiment(StopTime=10), Documentation(info="<html>
<p>
This example demonstrates a network of blocks
from package <a href=\"modelica://Modelica.Blocks.Interaction\">Modelica.Blocks.Interaction</a>
to show how diagram animations can be constructed.
</p>

</html>"));
  end Interaction1;

  model BusUsage "Demonstrates the usage of a signal bus"
    extends Modelica.Icons.Example;

    public
    Blocks.Sources.IntegerStep integerStep(
        height=1,
        offset=2,
        startTime=0.5)
        annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
    Blocks.Sources.BooleanStep booleanStep(startTime=0.5)
        annotation (Placement(transformation(extent={{-58,0},{-38,20}})));
    Blocks.Sources.Sine sine(f=1)
        annotation (Placement(transformation(extent={{-60,40},{-40,60}})));

    Blocks.Examples.BusUsage_Utilities.Part part
        annotation (Placement(transformation(extent={{-60,-80},{-40,-60}})));
    Blocks.Math.Gain gain(k=1)
        annotation (Placement(transformation(extent={{-40,70},{-60,90}})));
    protected
    BusUsage_Utilities.Interfaces.ControlBus controlBus annotation (Placement(
          transformation(
          origin={30,10},
          extent={{-20,20},{20,-20}},
          rotation=90)));
  equation

    connect(sine.y, controlBus.realSignal1) annotation (Line(
        points={{-39,50},{12,50},{12,14},{30,14},{30,10}}, color={0,0,127}));
    connect(booleanStep.y, controlBus.booleanSignal) annotation (Line(
        points={{-37,10},{30,10}}, color={255,0,255}));
    connect(integerStep.y, controlBus.integerSignal) annotation (Line(
        points={{-39,-30},{0,-30},{0,6},{32,6},{32,10},{30,10}}, color={255,127,0}));
    connect(part.subControlBus, controlBus.subControlBus) annotation (Line(
        points={{-40,-70},{30,-70},{30,10}},
        color={255,204,51},
        thickness=0.5));
    connect(gain.u, controlBus.realSignal1) annotation (Line(
        points={{-38,80},{20,80},{20,18},{32,18},{32,10},{30,10}}, color={0,0,127}));
    annotation (Documentation(info="<html>
<p><strong>Signal bus concept</strong></p>
<p>
In technical systems, such as vehicles, robots or satellites, many signals
are exchanged between components. In a simulation system, these signals
are usually modelled by signal connections of input/output blocks.
Unfortunately, the signal connection structure may become very complicated,
especially for hierarchical models.
</p>

<p>
The same is also true for real technical systems. To reduce complexity
and get higher flexibility, many technical systems use data buses to
exchange data between components. For the same reasons, it is often better
to use a \"signal bus\" concept also in a Modelica model. This is demonstrated
at hand of this model (Modelica.Blocks.Examples.BusUsage):
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/BusUsage.png\"
     alt=\"BusUsage.png\">

<ul>
<li> Connector instance \"controlBus\" is a hierarchical connector that is
     used to exchange signals between different components. It is
     defined as \"expandable connector\" in order that <strong>no</strong> central definition
     of the connector is needed but is automatically constructed by the
     signals connected to it (see also <a href=\"https://specification.modelica.org/v3.4/Ch9.html#expandable-connectors\">Section 9.1.3 (Expandable Connectors) of the Modelica 3.4 specification</a>).</li>
<li> Input/output signals can be directly connected to the \"controlBus\".</li>
<li> A component, such as \"part\", can be directly connected to the \"controlBus\",
     provided it has also a bus connector, or the \"part\" connector is a
     sub-connector contained in the \"controlBus\".</li>
</ul>

<p>
The control and sub-control bus icons are provided within Modelica.Icons.
In <a href=\"modelica://Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces\">Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces</a>
the buses for this example are defined. Both the \"ControlBus\" and the \"SubControlBus\" are
<strong>expandable</strong> connectors that do not define any variable. For example,
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces.ControlBus#text\">Interfaces.ControlBus</a>
is defined as:
</p>
<blockquote><pre>
<strong>expandable connector</strong> ControlBus
    <strong>extends</strong> Modelica.Icons.ControlBus;
    <strong>annotation</strong> ();
<strong>end</strong> ControlBus;
</pre></blockquote>
<p>
Note, the \"annotation\" in the connector is important since the color
and thickness of a connector line are taken from the first
line element in the icon annotation of a connector class. Above, a small rectangle in the
color of the bus is defined (and therefore this rectangle is not
visible). As a result, when connecting from an instance of this
connector to another connector instance, the connecting line has
the color of the \"ControlBus\" with double width (due to \"thickness=0.5\").
</p>

<p>
An <strong>expandable</strong> connector is a connector where the content of the connector
is constructed by the variables connected to instances of this connector.
For example, if \"sine.y\" is connected to the \"controlBus\", a pop-up menu may appear:
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/BusUsage2.png\"
     alt=\"BusUsage2.png\">

<p>
The \"Add variable/New name\" field allows the user to define the name of the signal on
the \"controlBus\". When typing \"realSignal1\" as \"New name\", a connection of the form:
</p>

<blockquote><pre>
<strong>connect</strong>(sine.y, controlBus.realSignal1)
</pre></blockquote>

<p>
is generated and the \"controlBus\" contains the new signal \"realSignal1\". Modelica tools
may give more support in order to list potential signals for a connection. Therefore, in
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage_Utilities.Interfaces\">BusUsage_Utilities.Interfaces</a>
the expected implementation of the \"ControlBus\" and of the \"SubControlBus\" are given.
For example \"Internal.ControlBus\" is defined as:
</p>

<blockquote><pre>
<strong>expandable connector</strong> StandardControlBus
  <strong>extends</strong> BusUsage_Utilities.Interfaces.ControlBus;

  <strong>import</strong> Modelica.Units.SI;
  SI.AngularVelocity    realSignal1   \"First Real signal\";
  SI.Velocity           realSignal2   \"Second Real signal\";
  Integer               integerSignal \"Integer signal\";
  Boolean               booleanSignal \"Boolean signal\";
  StandardSubControlBus subControlBus \"Combined signal\";
<strong>end</strong> StandardControlBus;
</pre></blockquote>

<p>
Consequently, when connecting now from \"sine.y\" to \"controlBus\", the menu
looks differently:
</p>

<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/BusUsage3.png\"
     alt=\"BusUsage3.png\">

<p>
Note, even if the signals from \"Internal.StandardControlBus\" are listed, these are
just potential signals. The user might still add different signal names.
</p>

</html>"), experiment(StopTime=2));
  end BusUsage;

  model Rectifier6pulseFFT "Example of FFT block"
    extends Modelica.Electrical.Machines.Examples.Transformers.Rectifier6pulse;
    Blocks.Math.RealFFT realFFT(
        startTime=0.04,
        f_max=2000,
        f_res=5,
        resultFileName="rectifier6pulseFFTresult.mat") annotation (Placement(
            transformation(extent={{-10,-10},{10,10}}, origin={-40,-20})));
  equation
    connect(currentSensor.i[1], realFFT.u)
      annotation (Line(points={{-70,-11},{-70,-20},{-52,-20}},
                                                     color={0,0,127}));
    annotation (experiment(StopTime=0.25, Interval=0.0001),
      Documentation(info="<html>
<p>
This example is based on a&nbsp;<a href=\"modelica://Modelica.Electrical.Machines.Examples.Transformers.Rectifier6pulse\">6-pulse rectifier example</a>,
calculating the harmonics with the <a href=\"modelica://Modelica.Blocks.Math.RealFFT\">FFT block</a>.
</p>
<p>
Sampling starts after the initial transients are settled - waiting for
<code>2&nbsp;periods&nbsp;= 2/f&nbsp;= 0.04&nbsp;s&nbsp;= realFFT.startTime</code>.
Choosing a&nbsp;maximum frequency <code>f_max&nbsp;=&nbsp;2000&nbsp;Hz</code>,
a&nbsp;frequency resolution <code>f_res&nbsp;=&nbsp;5&nbsp;Hz</code>
(both given in the block <code>realFFT</code>) and
the default oversampling factor <code>f_max_factor&nbsp;=&nbsp;5</code>,
we have to acquire <code>n&nbsp;= 2*f_max/f_res*f_max_factor&nbsp;=&nbsp;4000</code>
sampling intervals.
The resulting sampling interval is <code>samplePeriod&nbsp;=&nbsp;1/(n*f_res)&nbsp;=&nbsp;0.05&nbsp;ms</code>.
Thus, we have to sample for a&nbsp;period of <code>n*samplePeriod&nbsp;=&nbsp;1/f_res&nbsp;=&nbsp;0.2&nbsp;s</code>.
</p>
<p>
The result file &quot;rectifier6pulseFFTresult.mat&quot; can be used to plot
amplitudes versus frequencies.
Note that for each frequency three rows exit: one with amplitude zero,
one with the calculated amplitude, one with amplitude zero.
Thus, the second column (amplitude) can be easily plotted versus the first column (frequency).
As expected, one can see the 5<sup>th</sup>, 7<sup>th</sup>, 11<sup>th</sup>,
13<sup>th</sup>, 17<sup>th</sup>, 19<sup>th</sup>, 23<sup>th</sup>, 25<sup>th</sup>,
&hellip; harmonic in the result.
</p>
</html>"));
  end Rectifier6pulseFFT;

  model Rectifier12pulseFFT "Example of FFT block"
    extends Modelica.Electrical.Machines.Examples.Transformers.Rectifier12pulse;
    Blocks.Math.RealFFT realFFT(
        startTime=0.04,
        f_max=2000,
        f_res=5,
        resultFileName="rectifier12pulseFFTresult.mat") annotation (Placement(
            transformation(extent={{-10,-10},{10,10}}, origin={-40,-20})));
  equation
    connect(currentSensor.i[1], realFFT.u) annotation (Line(points={{-70,-11},{-70,-20},{-52,-20}},
                                   color={0,0,127}));
    annotation (experiment(StopTime=0.25, Interval=0.0001),
      Documentation(info="<html>
<p>
This example is based on a&nbsp;<a href=\"modelica://Modelica.Electrical.Machines.Examples.Transformers.Rectifier12pulse\">12-pulse rectifier example</a>,
calculating the harmonics with the <a href=\"modelica://Modelica.Blocks.Math.RealFFT\">FFT block</a>.
</p>
<p>
Sampling starts after the initial transients are settled - waiting for
<code>2&nbsp;periods&nbsp;= 2/f&nbsp;= 0.04&nbsp;s&nbsp;= realFFT.startTime</code>.
Choosing a&nbsp;maximum frequency <code>f_max&nbsp;=&nbsp;2000&nbsp;Hz</code>,
a&nbsp;frequency resolution <code>f_res&nbsp;=&nbsp;5&nbsp;Hz</code>
(both given in the block <code>realFFT</code>) and
the default oversampling factor <code>f_max_factor&nbsp;=&nbsp;5</code>,
we have to acquire <code>n&nbsp;= 2*f_max/f_res*f_max_factor&nbsp;=&nbsp;4000</code>
sampling intervals.
The resulting sampling interval is <code>samplePeriod&nbsp;=&nbsp;1/(n*f_res)&nbsp;=&nbsp;0.05&nbsp;ms</code>.
Thus, we have to sample for a&nbsp;period of <code>n*samplePeriod = 1/f_res = 0.2 s</code>.
</p>
<p>
The resultfile &quot;rectifier12pulseFFTresult.mat&quot; can be used to plot amplitudes versus frequencies.
Note that for each frequency three rows exit: one with amplitude zero,
one with the calculated amplitude, one with amplitude zero.
Thus, the second column (amplitude) can be easily plotted versus the first column (frequency).
As expected, one can see the 11<sup>th</sup>, 13<sup>th</sup>, 23<sup>th</sup>, 25<sup>th</sup>,
&hellip; harmonic in the result.
</p>
</html>"));
  end Rectifier12pulseFFT;

  model TotalHarmonicDistortion "Calculation of total harmonic distortion of voltage"
    extends Modelica.Icons.Example;
    parameter SI.Frequency f1 = 50 "Fundamental wave frequency";
    parameter SI.Voltage V1 = 100 "Fundamental wave RMS voltage";
    parameter SI.Voltage V3 = 20 "Third harmonic wave RMS voltage";
    final parameter Real THD1 = V3/V1 "Theoretically obtained THD with respect to fundamental wave";
    final parameter Real THDrms = V3/sqrt(V1^2+V3^2) "Theoretically obtained THD with respect to RMS";
    Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(transformation(extent={{-50,-60},{-30,-40}})));
    Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage3(V=sqrt(2)*V3, f=3*f1,
      startTime=0.02)                                           annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-40,10})));
    Modelica.Electrical.Analog.Sources.SineVoltage sineVoltage1(V=sqrt(2)*V1, f=f1,
      startTime=0.02)                                           annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-40,-20})));
    Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=270)));
    Blocks.Math.TotalHarmonicDistortion thd1(f=f1)
        annotation (Placement(transformation(extent={{30,10},{50,30}})));
    Blocks.Math.TotalHarmonicDistortion thdRMS(f=f1, useFirstHarmonic=false)
        annotation (Placement(transformation(extent={{30,-30},{50,-10}})));
  equation
    connect(voltageSensor.p, sineVoltage3.p) annotation (Line(points={{0,10},{0,30},{-40,30},{-40,20}}, color={0,0,255}));
    connect(sineVoltage3.n, sineVoltage1.p) annotation (Line(points={{-40,0},{-40,-10}}, color={0,0,255}));
    connect(sineVoltage1.n, ground.p) annotation (Line(points={{-40,-30},{-40,-40}}, color={0,0,255}));
    connect(ground.p, voltageSensor.n) annotation (Line(points={{-40,-40},{-40,-30},{0,-30},{0,-10}}, color={0,0,255}));
    connect(thd1.u, voltageSensor.v) annotation (Line(points={{28,20},{20,20},{20,0},{11,0}}, color={0,0,127}));
    connect(voltageSensor.v, thdRMS.u) annotation (Line(points={{11,0},{20,0},{20,-20},{28,-20}}, color={0,0,127}));
    annotation (experiment(
        StopTime=0.1,
        Interval=0.0001,
        Tolerance=1e-06), Documentation(info="<html>
<p>This example compares the result of the
<a href=\"modelica://Modelica.Blocks.Math.TotalHarmonicDistortion\">total harmonic distortion (THD)</a>
with respect to the fundamental wave and with respect to the total root mean square (RMS).
In this simulation model a non-sinusoidal voltage
wave form is created by the superposition two voltage waves:</p>

<ul>
<li>Fundamental wave with RMS voltage <code>V1</code> and frequency <code>f1</code></li>
<li>Third harmonic wave with RMS voltage <code>V3</code> and frequency <code>3*f1</code></li>
</ul>

<p>This simulation model compares numerically determined THD values with results, obtained by
theoretical calculations:</p>

<ul>
<li>Compare the numerically determined THD value <code>thd1.y</code> and the theoretical value <code>THD1</code>,
    both with respect to the fundamental wave; also plot <code>thd1.valid</code></li>
<li>Compare the numerically determined THD value <code>thdRMS.y</code> and the theoretical value <code>THDrms</code>,
    both with respect to the RMS value; also plot <code>thdRMS.valid</code></li>
</ul>
</html>"));
  end TotalHarmonicDistortion;

  model Modulation "Demonstrate amplitude modulation an frequency modulation"
    extends Modelica.Icons.Example;
    Blocks.Sources.SineVariableFrequencyAndAmplitude sine(
        useConstantAmplitude=true,
        useConstantFrequency=true,
        constantFrequency=100,
        phi(fixed=true))
        annotation (Placement(transformation(extent={{-10,60},{10,80}})));
    Blocks.Sources.Sine amplitude(
        amplitude=0.5,
        f=2,
        offset=1)
        annotation (Placement(transformation(extent={{-52,20},{-32,40}})));
    Blocks.Sources.SineVariableFrequencyAndAmplitude sinAM(
        useConstantAmplitude=false,
        useConstantFrequency=true,
        constantFrequency=100,
        phi(fixed=true))
        annotation (Placement(transformation(extent={{-10,20},{10,40}})));
    Sources.CosineVariableFrequencyAndAmplitude cosAM(
      useConstantAmplitude=false,
      useConstantFrequency=true,
      constantFrequency=100,
      phi(fixed=true))
      annotation (Placement(transformation(extent={{-10,-12},{10,8}})));
    Blocks.Sources.Sine frequency(
        amplitude=50,
        f=2,
        offset=100)
        annotation (Placement(transformation(extent={{-50,-50},{-30,-30}})));
    Blocks.Sources.SineVariableFrequencyAndAmplitude sinFM(
        useConstantAmplitude=true,
        useConstantFrequency=false,
        constantFrequency=100,
        phi(fixed=true))
        annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));
    Sources.CosineVariableFrequencyAndAmplitude cosFM(
      useConstantAmplitude=true,
      useConstantFrequency=false,
      constantFrequency=100,
      phi(fixed=true))
      annotation (Placement(transformation(extent={{-10,-80},{10,-60}})));
  equation
    connect(amplitude.y, sinAM.amplitude) annotation (Line(points={{-31,30},{-20,30},
            {-20,36},{-12,36}}, color={0,0,127}));
    connect(frequency.y, sinFM.f) annotation (Line(points={{-29,-40},{-20,-40},{-20,
            -46},{-12,-46}}, color={0,0,127}));
    connect(amplitude.y, cosAM.amplitude) annotation (Line(points={{-31,30},{-20,30},
            {-20,4},{-12,4}}, color={0,0,127}));
    connect(frequency.y, cosFM.f) annotation (Line(points={{-29,-40},{-20,-40},{-20,
            -76},{-12,-76}}, color={0,0,127}));
    annotation (experiment(StopTime=1.0, Interval=0.0001), Documentation(info="<html>
<p>
This example demonstrates amplitude modulation (AM) and frequency modulation (FM).
</p>
</html>"));
  end Modulation;

  model SinCosEncoder "Evaluation of a sinusoidal encoder"
    extends Modelica.Icons.Example;
    import Modelica.Constants.pi;
    SI.AngularVelocity w=2*pi*ramp.y "2*pi*f";
    Sources.Ramp ramp(
      height=100,
      duration=1,
      offset=0,
      startTime=0)
      annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    Sources.CosineVariableFrequencyAndAmplitude
                                              cosB(
      useConstantAmplitude=true,
      offset=1.5,
      phi(fixed=true))
      annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
    Sources.CosineVariableFrequencyAndAmplitude
                                              cosBminus(
      useConstantAmplitude=true,
      constantAmplitude=-1,
      offset=1.5,
      phi(fixed=true))
      annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
    Sources.SineVariableFrequencyAndAmplitude sinA(
      useConstantAmplitude=true,
      offset=1.5,
      phi(fixed=true))
      annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
    Sources.SineVariableFrequencyAndAmplitude sinAminus(
      useConstantAmplitude=true,
      constantAmplitude=-1,
      offset=1.5,
      phi(fixed=true))
      annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
    Math.Feedback feedbackCos
      annotation (Placement(transformation(extent={{-30,40},{-10,60}})));
    Math.Feedback feedbackSin
      annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
    Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator
        annotation (Placement(transformation(extent={{10,-10},{30,10}})));
    Continuous.Integrator integrator(k=1e6)
      annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    Continuous.Der der1
      annotation (Placement(transformation(extent={{80,-10},{100,10}})));
    Math.WrapAngle wrapAngle(positiveRange=false)
      annotation (Placement(transformation(extent={{80,20},{100,40}})));
    Modelica.Electrical.Machines.SpacePhasors.Blocks.ToPolar toPolar
        annotation (Placement(transformation(extent={{10,20},{30,40}})));
  equation
    connect(ramp.y, sinA.f) annotation (Line(points={{-79,0},{-70,0},{-70,-26},{-62,
            -26}},color={0,0,127}));
    connect(ramp.y, sinAminus.f) annotation (Line(points={{-79,0},{-70,0},{-70,-56},
            {-62,-56}}, color={0,0,127}));
    connect(ramp.y, cosBminus.f) annotation (Line(points={{-79,0},{-70,0},{-70,14},
            {-62,14}}, color={0,0,127}));
    connect(ramp.y, cosB.f) annotation (Line(points={{-79,0},{-70,0},{-70,44},{-62,
            44}}, color={0,0,127}));
    connect(cosBminus.y, feedbackCos.u2)
      annotation (Line(points={{-39,20},{-20,20},{-20,42}}, color={0,0,127}));
    connect(cosB.y, feedbackCos.u1)
      annotation (Line(points={{-39,50},{-28,50}}, color={0,0,127}));
    connect(sinA.y, feedbackSin.u1)
      annotation (Line(points={{-39,-20},{-28,-20}}, color={0,0,127}));
    connect(sinAminus.y, feedbackSin.u2)
      annotation (Line(points={{-39,-50},{-20,-50},{-20,-28}}, color={0,0,127}));
    connect(feedbackCos.y, rotator.u[1])
      annotation (Line(points={{-11,50},{0,50},{0,0},{8,0}}, color={0,0,127}));
    connect(feedbackSin.y, rotator.u[2])
      annotation (Line(points={{-11,-20},{0,-20},{0,0},{8,0}}, color={0,0,127}));
    connect(rotator.y[2], integrator.u)
      annotation (Line(points={{31,0},{38,0}}, color={0,0,127}));
    connect(integrator.y, rotator.angle) annotation (Line(points={{61,0},{70,0},{70,
            -20},{20,-20},{20,-12}}, color={0,0,127}));
    connect(integrator.y, der1.u)
      annotation (Line(points={{61,0},{78,0}}, color={0,0,127}));
    connect(integrator.y, wrapAngle.u)
      annotation (Line(points={{61,0},{70,0},{70,30},{78,30}}, color={0,0,127}));
    connect(feedbackCos.y, toPolar.u[1])
      annotation (Line(points={{-11,50},{0,50},{0,30},{8,30}}, color={0,0,127}));
    connect(feedbackSin.y, toPolar.u[2]) annotation (Line(points={{-11,-20},{0,-20},
            {0,30},{8,30}}, color={0,0,127}));
    annotation (experiment(StopTime=1.0, Interval=5e-05, Tolerance=1e-05), Documentation(info="<html>
<p>
This examples demonstrates robust evaluation of a sin-cos-encoder.
</p>
<p>
The sin-cos-encoder provides four tracks:
</p>
<ul>
<li>cosine</li>
<li>minus sine</li>
<li>sine</li>
<li>minus cosine</li>
</ul>
<p>
All four tracks have the same amplitude and the same offset &gt; amplitude. Offset is used to detect loss of a track.
To remove offset, (minus sine) is subtracted from (sine) and (minus cosine) from (cosine),
resulting in a cosine and a sine signal with doubled amplitude but without offset.
</p>
<p>
Interpreting cosine and sine as real and imaginary part of a phasor, one could calculate the angle of the phasor (i.e. transform rectangular coordinates to polar coordinates).
This is not very robust if the signals are superimposed with some noise.
Therefore the phasor is rotated by an angle that is obtained by a controller. The controller aims at imaginary part equal to zero.
The resulting angle is continuous, i.e. differentiating the angle results in 2*&pi;*frequency.
If desired, the angle can be wrapped to the interval [-&pi;, +&pi;].
</p>
</html>"));
  end SinCosEncoder;

  model CompareSincExpSine "Compare sinc and exponential sine signal"
    extends Modelica.Icons.Example;
    Sources.Sinc sinc(f=5)
      annotation (Placement(transformation(extent={{-10,20},{10,40}})));
    Sources.ExpSine expSine1(f=5, damping=5)
      annotation (Placement(transformation(extent={{-10,-20},{10,0}})));
    Sources.ExpSine expSine2(
      f=5,
      phase=Modelica.Constants.pi/2,
      damping=5)
      annotation (Placement(transformation(extent={{-10,-60},{10,-40}})));
    annotation (experiment(StopTime=1.0, Interval=0.0001), Documentation(info="<html>
<p>
Compare the sinc signal and an exponentially damped sine.
</p>
</html>"));
  end CompareSincExpSine;

  package Noise "Library of examples to demonstrate the usage of package Blocks.Noise"
    extends Modelica.Icons.ExamplesPackage;

    model UniformNoise
      "Demonstrates the most simple usage of the UniformNoise block"
      extends Modelica.Icons.Example;
      output Real uniformNoise2_y = uniformNoise2.y;

      inner Blocks.Noise.GlobalSeed globalSeed
          annotation (Placement(transformation(extent={{-20,40},{0,60}})));
      Blocks.Noise.UniformNoise uniformNoise1(
          samplePeriod=0.02,
          y_min=-1,
          y_max=3)
          annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
      Blocks.Noise.UniformNoise uniformNoise2(
          samplePeriod=0.02,
          y_min=-1,
          y_max=3,
          useAutomaticLocalSeed=false,
          fixedLocalSeed=10)
          annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
     annotation (experiment(StopTime=2), Documentation(info="<html>
<p>
This example demonstrates the most simple usage of the
<a href=\"modelica://Modelica.Blocks.Noise.UniformNoise\">Noise.UniformNoise</a>
block:
</p>

<ul>
<li> <strong>globalSeed</strong> is the <a href=\"modelica://Modelica.Blocks.Noise.GlobalSeed\">Noise.GlobalSeed</a>
     block with default options (just dragged from sublibrary Noise).</li>
<li> <strong>uniformNoise1</strong> is an instance of
     <a href=\"modelica://Modelica.Blocks.Noise.UniformNoise\">Noise.UniformNoise</a> with
     samplePeriod = 0.02 s and a Uniform distribution with limits y_min=-1, y_max=3.</li>
<li> <strong>uniformNoise2</strong> is identical to uniformNoise1 with the exception that
      useAutomaticLocalSeed=false and fixedLocalSeed=10.</li>
</ul>

<p>
At every 0.02 seconds a time event occurs and a uniform random number in the band between
-1 ... 3 is drawn. This random number is held constant until the next sample instant.
The result of a simulation is shown in the next diagram:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/UniformNoise.png\">
</blockquote>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end UniformNoise;

    model AutomaticSeed
      "Demonstrates noise with startTime and automatic local seed for UniformNoise"
       extends Modelica.Icons.Example;
       parameter SI.Time startTime = 0.5 "Start time of noise";
       parameter Real y_off = -1.0 "Output of block before startTime";

       output Real manualSeed1_y = manualSeed1.y;
       output Real manualSeed2_y = manualSeed2.y;
       output Real manualSeed3_y = manualSeed3.y;

      inner Blocks.Noise.GlobalSeed globalSeed(useAutomaticSeed=false,
            enableNoise=true)
          annotation (Placement(transformation(extent={{60,60},{80,80}})));

      Blocks.Noise.UniformNoise automaticSeed1(
          samplePeriod=0.01,
          startTime=startTime,
          y_off=y_off,
          y_min=-1,
          y_max=3)
          annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
      Blocks.Noise.UniformNoise automaticSeed2(
          samplePeriod=0.01,
          startTime=startTime,
          y_off=y_off,
          y_min=-1,
          y_max=3)
          annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
      Blocks.Noise.UniformNoise automaticSeed3(
          samplePeriod=0.01,
          startTime=startTime,
          y_off=y_off,
          y_min=-1,
          y_max=3)
          annotation (Placement(transformation(extent={{-60,-60},{-40,-40}})));
      Blocks.Noise.UniformNoise manualSeed1(
          samplePeriod=0.01,
          startTime=startTime,
          y_off=y_off,
          useAutomaticLocalSeed=false,
          fixedLocalSeed=1,
          y_min=-1,
          y_max=3,
          enableNoise=true)
          annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Blocks.Noise.UniformNoise manualSeed2(
          samplePeriod=0.01,
          startTime=startTime,
          y_off=y_off,
          useAutomaticLocalSeed=false,
          fixedLocalSeed=2,
          y_min=-1,
          y_max=3)
          annotation (Placement(transformation(extent={{0,-20},{20,0}})));
      Blocks.Noise.UniformNoise manualSeed3(
          samplePeriod=0.01,
          startTime=startTime,
          y_off=y_off,
          useAutomaticLocalSeed=false,
          y_min=-1,
          y_max=3,
          fixedLocalSeed=3)
          annotation (Placement(transformation(extent={{0,-60},{20,-40}})));
     annotation (experiment(StopTime=2), Documentation(info="<html>
<p>
This example demonstrates manual and automatic seed selection of
<a href=\"modelica://Modelica.Blocks.Noise.UniformNoise\">UniformNoise</a> blocks, as well
as starting the noise at startTime = 0.5 s with an output value of y = -1 before this
time. All noise blocks in this example generate uniform noise in the
band y_min=-1 .. y_max=3 with samplePeriod = 0.01 s.
</p>

<p>
The blocks automaticSeed1, automaticSeed2, automaticSeed3 use the default
option to automatically initialize the pseudo random number generators
of the respective block. As a result, different noise is generated, see next
diagram:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/AutomaticSeed1.png\">
</blockquote>

<p>
The blocks manualSeed1, manualSeed2, manualSeed3 use manual selection of the local seed
(useAutomaticLocalSeed = false). They use a fixedLocalSeed of 1, 2, and 3 respectively.
Again, different noise is generated, see next diagram:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/AutomaticSeed2.png\">
</blockquote>

<p>
Try to set fixedLocalSeed = 1 in block manualSeed2. As a result, the blocks manualSeed1 and
manualSeed2 will produce exactly the same noise.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end AutomaticSeed;

    model Distributions
      "Demonstrates noise with different types of distributions"
      extends Modelica.Icons.Example;
      parameter SI.Period samplePeriod=0.02
        "Sample period of all blocks";
      parameter Real y_min = -1 "Minimum value of band for random values";
      parameter Real y_max = 3 "Maximum value of band for random values";
      inner Blocks.Noise.GlobalSeed globalSeed(useAutomaticSeed=false)
          annotation (Placement(transformation(extent={{40,60},{60,80}})));
      output Real uniformNoise_y =         uniformNoise.y;
      output Real truncatedNormalNoise_y = truncatedNormalNoise.y;

      Integer n=if time < 0.5 then 12 else 2;

      Blocks.Noise.UniformNoise uniformNoise(
          useAutomaticLocalSeed=false,
          fixedLocalSeed=1,
          samplePeriod=samplePeriod,
          y_min=y_min,
          y_max=y_max)
          annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
      Blocks.Noise.TruncatedNormalNoise truncatedNormalNoise(
          useAutomaticLocalSeed=false,
          fixedLocalSeed=1,
          samplePeriod=samplePeriod,
          y_min=y_min,
          y_max=y_max)
          annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
     annotation (experiment(StopTime=2), Documentation(info="<html>
<p>
This example demonstrates different noise distributions methods that can be selected
for a Noise block. Both noise blocks use samplePeriod = 0.02 s, y_min=-1, y_max=3, and have
identical fixedLocalSeed. This means that the same random numbers are drawn for the blocks.
However, the random numbers are differently transformed according to the selected distributions
(uniform and truncated normal distribution), and therefore the blocks have different output values.
Simulation results are shown in the next diagram:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/Distributions.png\">
</blockquote>

<p>
As can be seen, uniform noise is distributed evenly between -1 and 3, and
truncated normal distribution has more values centered around the mean value 1.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end Distributions;

    model UniformNoiseProperties
      "Demonstrates the computation of properties for uniformly distributed noise"
      extends Modelica.Icons.Example;
      parameter Real y_min = 0 "Minimum value of band";
      parameter Real y_max = 6 "Maximum value of band";
      parameter Real pMean = (y_min + y_max)/2
        "Theoretical mean value of uniform distribution";
      parameter Real var =  (y_max - y_min)^2/12
        "Theoretical variance of uniform distribution";
      parameter Real std =  sqrt(var)
        "Theoretical standard deviation of uniform distribution";
      inner Blocks.Noise.GlobalSeed globalSeed
          annotation (Placement(transformation(extent={{80,60},{100,80}})));
      output Real meanError_y = meanError.y;
      output Real sigmaError_y = sigmaError.y;

      Blocks.Noise.UniformNoise noise(
          samplePeriod=0.001,
          y_min=y_min,
          y_max=y_max,
          useAutomaticLocalSeed=false)
          annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
      Blocks.Math.ContinuousMean mean
          annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
      Blocks.Math.Variance variance
          annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
      Blocks.Math.MultiProduct theoreticalVariance(nu=2)
          annotation (Placement(transformation(extent={{28,-36},{40,-24}})));
      Blocks.Math.Feedback meanError
          annotation (Placement(transformation(extent={{40,60},{60,80}})));
      Blocks.Sources.Constant theoreticalMean(k=pMean)
          annotation (Placement(transformation(extent={{-10,40},{10,60}})));
      Blocks.Math.Feedback varianceError
          annotation (Placement(transformation(extent={{40,0},{60,20}})));
      Blocks.Sources.Constant theoreticalSigma(k=std)
          annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
      Blocks.Math.StandardDeviation standardDeviation
          annotation (Placement(transformation(extent={{-40,-80},{-20,-60}})));
      Blocks.Math.Feedback sigmaError
          annotation (Placement(transformation(extent={{40,-60},{60,-80}})));
    equation
      connect(noise.y, mean.u) annotation (Line(
          points={{-59,70},{-42,70}}, color={0,0,127}));
      connect(noise.y, variance.u) annotation (Line(
          points={{-59,70},{-52,70},{-52,10},{-42,10}}, color={0,0,127}));
      connect(mean.y, meanError.u1) annotation (Line(
          points={{-19,70},{42,70}}, color={0,0,127}));
      connect(theoreticalMean.y, meanError.u2) annotation (Line(
          points={{11,50},{50,50},{50,62}}, color={0,0,127}));
      connect(theoreticalSigma.y, theoreticalVariance.u[1]) annotation (Line(
          points={{11,-30},{24,-30},{24,-27.9},{28,-27.9}}, color={0,0,127}));
      connect(theoreticalSigma.y, theoreticalVariance.u[2]) annotation (Line(
          points={{11,-30},{24,-30},{24,-32.1},{28,-32.1}}, color={0,0,127}));
      connect(variance.y, varianceError.u1) annotation (Line(
          points={{-19,10},{42,10}}, color={0,0,127}));
      connect(theoreticalVariance.y, varianceError.u2) annotation (Line(
          points={{41.02,-30},{50,-30},{50,2}}, color={0,0,127}));
      connect(noise.y, standardDeviation.u) annotation (Line(
          points={{-59,70},{-52,70},{-52,-70},{-42,-70}}, color={0,0,127}));
      connect(standardDeviation.y, sigmaError.u1) annotation (Line(
          points={{-19,-70},{42,-70}}, color={0,0,127}));
      connect(theoreticalSigma.y, sigmaError.u2) annotation (Line(
          points={{11,-30},{18,-30},{18,-42},{50,-42},{50,-62}}, color={0,0,127}));
     annotation (experiment(StopTime=20, Interval=0.4e-2, Tolerance=1e-009),
        Documentation(info="<html>
<p>
This example demonstrates statistical properties of the
<a href=\"modelica://Modelica.Blocks.Noise.UniformNoise\">Blocks.Noise.UniformNoise</a> block
using a <strong>uniform</strong> random number distribution.
Block &quot;noise&quot; defines a band of 0 .. 6 and from the generated noise the mean and the variance
is computed with blocks of package <a href=\"modelica://Modelica.Blocks.Math\">Blocks.Math</a>.
Simulation results are shown in the next diagram:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/UniformNoiseProperties1.png\"/>
</blockquote>

<p>
The mean value of a uniform noise in the range 0 .. 6 is 3 and its variance is
3 as well. The simulation results above show good agreement (after a short initial phase).
This demonstrates that the random number generator and the mapping to a uniform
distribution have good statistical properties.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end UniformNoiseProperties;

    model NormalNoiseProperties
      "Demonstrates the computation of properties for normally distributed noise"
      extends Modelica.Icons.Example;
      parameter Real mu = 3 "Mean value for normal distribution";
      parameter Real sigma = 1 "Standard deviation for normal distribution";
      parameter Real pMean = mu "Theoretical mean value of normal distribution";
      parameter Real var =  sigma^2
        "Theoretical variance of uniform distribution";
      parameter Real std =  sigma
        "Theoretical standard deviation of normal distribution";
      inner Blocks.Noise.GlobalSeed globalSeed
          annotation (Placement(transformation(extent={{80,60},{100,80}})));
      output Real meanError_y = meanError.y;
      output Real sigmaError_y = sigmaError.y;

      Blocks.Noise.NormalNoise noise(
          samplePeriod=0.001,
          mu=mu,
          sigma=sigma,
          useAutomaticLocalSeed=false)
          annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
      Blocks.Math.ContinuousMean mean
          annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
      Blocks.Math.Variance variance
          annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
      Blocks.Math.MultiProduct theoreticalVariance(nu=2)
          annotation (Placement(transformation(extent={{28,-36},{40,-24}})));
      Blocks.Math.Feedback meanError
          annotation (Placement(transformation(extent={{40,60},{60,80}})));
      Blocks.Sources.Constant theoreticalMean(k=pMean)
          annotation (Placement(transformation(extent={{-10,40},{10,60}})));
      Blocks.Math.Feedback varianceError
          annotation (Placement(transformation(extent={{40,0},{60,20}})));
      Blocks.Sources.Constant theoreticalSigma(k=std)
          annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
      Blocks.Math.StandardDeviation standardDeviation
          annotation (Placement(transformation(extent={{-40,-80},{-20,-60}})));
      Blocks.Math.Feedback sigmaError
          annotation (Placement(transformation(extent={{40,-60},{60,-80}})));
    equation
      connect(noise.y, mean.u) annotation (Line(
          points={{-59,70},{-42,70}}, color={0,0,127}));
      connect(noise.y, variance.u) annotation (Line(
          points={{-59,70},{-52,70},{-52,10},{-42,10}}, color={0,0,127}));
      connect(mean.y, meanError.u1) annotation (Line(
          points={{-19,70},{42,70}}, color={0,0,127}));
      connect(theoreticalMean.y, meanError.u2) annotation (Line(
          points={{11,50},{50,50},{50,62}}, color={0,0,127}));
      connect(theoreticalSigma.y, theoreticalVariance.u[1]) annotation (Line(
          points={{11,-30},{24,-30},{24,-27.9},{28,-27.9}}, color={0,0,127}));
      connect(theoreticalSigma.y, theoreticalVariance.u[2]) annotation (Line(
          points={{11,-30},{24,-30},{24,-32.1},{28,-32.1}}, color={0,0,127}));
      connect(variance.y, varianceError.u1) annotation (Line(
          points={{-19,10},{42,10}}, color={0,0,127}));
      connect(theoreticalVariance.y, varianceError.u2) annotation (Line(
          points={{41.02,-30},{50,-30},{50,2}}, color={0,0,127}));
      connect(noise.y, standardDeviation.u) annotation (Line(
          points={{-59,70},{-52,70},{-52,-70},{-42,-70}}, color={0,0,127}));
      connect(standardDeviation.y, sigmaError.u1) annotation (Line(
          points={{-19,-70},{42,-70}}, color={0,0,127}));
      connect(theoreticalSigma.y, sigmaError.u2) annotation (Line(
          points={{11,-30},{18,-30},{18,-42},{50,-42},{50,-62}}, color={0,0,127}));
     annotation (experiment(StopTime=20, Interval=0.4e-2, Tolerance=1e-009),
    Documentation(info="<html>
<p>
This example demonstrates statistical properties of the
<a href=\"modelica://Modelica.Blocks.Noise.NormalNoise\">Blocks.Noise.NormalNoise</a> block
using a <strong>normal</strong> random number distribution with mu=3, sigma=1.
From the generated noise the mean and the variance
is computed with blocks of package <a href=\"modelica://Modelica.Blocks.Math\">Blocks.Math</a>.
Simulation results are shown in the next diagram:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/NormalNoiseProperties1.png\">
</blockquote>

<p>
The mean value of a normal noise with mu=3 is 3 and the variance of normal noise
is sigma^2, so 1. The simulation results above show good agreement (after a short initial phase).
This demonstrates that the random number generator and the mapping to a normal
distribution have good statistical properties.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end NormalNoiseProperties;

    model Densities
      "Demonstrates how to compute distribution densities (= Probability Density Function)"
      extends Modelica.Icons.Example;
      output Real uniformDensity_y = uniformDensity.y;
      output Real normalDensity_y = normalDensity.y;
      output Real weibullDensity_y = weibullDensity.y;

      Utilities.UniformDensity
                        uniformDensity(u_min=-4, u_max=4)
        annotation (Placement(transformation(extent={{10,20},{30,40}})));
      Blocks.Sources.ContinuousClock clock
          annotation (Placement(transformation(extent={{-80,10},{-60,30}})));
      Blocks.Sources.Constant const(k=-10)
          annotation (Placement(transformation(extent={{-80,-30},{-60,-10}})));
      Blocks.Math.Add add
          annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
      Utilities.NormalDensity
                        normalDensity(mu=0, sigma=2)
        annotation (Placement(transformation(extent={{10,-10},{30,10}})));
      Utilities.WeibullDensity
                        weibullDensity(lambda=3, k=1.5)
        annotation (Placement(transformation(extent={{10,-40},{30,-20}})));
    equation
      connect(clock.y, add.u1) annotation (Line(
      points={{-59,20},{-53.5,20},{-53.5,6},{-48,6}}, color={0,0,127}));
      connect(const.y, add.u2) annotation (Line(
      points={{-59,-20},{-54,-20},{-54,-6},{-48,-6}}, color={0,0,127}));
      connect(add.y, uniformDensity.u) annotation (Line(
      points={{-25,0},{-14,0},{-14,30},{8,30}}, color={0,0,127}));
      connect(add.y, normalDensity.u) annotation (Line(
      points={{-25,0},{8,0}}, color={0,0,127}));
      connect(add.y, weibullDensity.u) annotation (Line(
      points={{-25,0},{-14,0},{-14,-30},{8,-30}}, color={0,0,127}));
     annotation (experiment(StopTime=20, Interval=2e-2),
        Documentation(info="<html>
<p>
This example demonstrates how to compute the probability density functions (pdfs) of
various distributions.
In the following diagram simulations results for the uniform, normal, and Weibull distribution
are shown. The outputs of the blocks are the pdfs that are plotted over one of the
inputs:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/Densities.png\">
</blockquote>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end Densities;

    model ImpureGenerator
      "Demonstrates the usage of the impure random number generator"
      extends Modelica.Icons.Example;
      output Real impureRandom_y = impureRandom.y;

      inner Blocks.Noise.GlobalSeed globalSeed(useAutomaticSeed=false)
          annotation (Placement(transformation(extent={{20,40},{40,60}})));

      Utilities.ImpureRandom impureRandom(samplePeriod=0.01)
        annotation (Placement(transformation(extent={{-60,20},{-40,40}})));
     annotation (experiment(StopTime=2), Documentation(info="<html>
<p>
This example demonstrates how to use the
<a href=\"modelica://Modelica.Math.Random.Utilities.impureRandom\">impureRandom(..)</a> function
to generate random values at event instants. Typically, this approach is only
used when implementing an own, specialized block that needs a random number
generator. Simulation results are shown in the next figure:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/ImpureGenerator.png\">
</blockquote>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end ImpureGenerator;

    model ActuatorWithNoise
      "Demonstrates how to model measurement noise in an actuator"
    extends Modelica.Icons.Example;
      Utilities.Parts.MotorWithCurrentControl motor
        annotation (Placement(transformation(extent={{-86,-10},{-66,10}})));
      Utilities.Parts.Controller controller
        annotation (Placement(transformation(extent={{-60,40},{-80,60}})));
      Blocks.Sources.Step speed(startTime=0.5, height=50)
          annotation (Placement(transformation(extent={{20,40},{0,60}})));
      Modelica.Mechanics.Rotational.Components.Gearbox gearbox(
        lossTable=[0,0.85,0.8,0.1,0.1],
        c=1e6,
        d=1e4,
        ratio=10,
        w_rel(fixed=true),
        b=0.0017453292519943,
        phi_rel(fixed=true))
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Modelica.Mechanics.Translational.Components.IdealGearR2T idealGearR2T(ratio=
            300) annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
      Modelica.Mechanics.Translational.Components.Mass mass(m=100)
        annotation (Placement(transformation(extent={{50,-10},{70,10}})));
      Modelica.Mechanics.Translational.Sources.ConstantForce constantForce(
          f_constant=10000) annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            origin={86,0})));
      Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising=50)
          annotation (Placement(transformation(extent={{-20,40},{-40,60}})));
      Modelica.Mechanics.Translational.Components.Mass rodMass(m=3)
        annotation (Placement(transformation(extent={{-4,-10},{16,10}})));
      Modelica.Mechanics.Translational.Components.SpringDamper elastoGap(c=1e8, d=
            1e5,
        v_rel(fixed=true),
        s_rel(fixed=true))
                 annotation (Placement(transformation(extent={{22,-10},{42,10}})));
      inner .Blocks.Noise.GlobalSeed globalSeed(enableNoise=true)
          annotation (Placement(transformation(extent={{60,60},{80,80}})));
    equation
      connect(controller.y1, motor.iq_rms1) annotation (Line(
          points={{-81,50},{-94,50},{-94,6},{-88,6}}, color={0,0,127}));
      connect(motor.phi, controller.positionMeasured) annotation (Line(
          points={{-71,8},{-66,8},{-66,20},{-50,20},{-50,44},{-58,44}}, color={0,0,127}));
      connect(motor.flange, gearbox.flange_a) annotation (Line(
          points={{-66,0},{-60,0}}));
      connect(gearbox.flange_b, idealGearR2T.flangeR) annotation (Line(
          points={{-40,0},{-32,0}}));
      connect(constantForce.flange, mass.flange_b) annotation (Line(
          points={{76,0},{70,0}}, color={0,127,0}));
      connect(speed.y, slewRateLimiter.u) annotation (Line(
          points={{-1,50},{-18,50}}, color={0,0,127}));
      connect(slewRateLimiter.y, controller.positionReference) annotation (Line(
          points={{-41,50},{-50,50},{-50,56},{-58,56}}, color={0,0,127}));
      connect(rodMass.flange_a, idealGearR2T.flangeT) annotation (Line(
          points={{-4,0},{-12,0}}, color={0,127,0}));
      connect(rodMass.flange_b, elastoGap.flange_a) annotation (Line(
          points={{16,0},{22,0}}, color={0,127,0}));
      connect(elastoGap.flange_b, mass.flange_a) annotation (Line(
          points={{42,0},{50,0}}, color={0,127,0}));
      annotation (
        experiment(StopTime=8, Interval = 0.01, Tolerance=1e-005),
        Documentation(info="<html>
<p>
This example models an actuator with a noisy sensor (which is in the motor component):
</p>

<blockquote>
<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/ActuatorNoiseDiagram.png\"/>
</p></blockquote>

<p>
The drive train consists of a synchronous motor with a current controller (= motor) and a gear box.
The gearbox drives a rod through a linear translation model. Softly attached to the rod is
another mass representing the actual actuator (= mass). The actuator is loaded with a constant force.
</p>

<p>
The whole drive is steered by a rate limited speed step command through a controller model.
In the motor the shaft angle is measured and this measurement signal is modelled by adding
additive noise to the motor angle.
</p>

<p>
In the following figure, the position of the actuator and the motor output torque are
shown with and without noise. The noise is not very strong, such that it has no visible effect
on the position of the actuator. The effect of the noise can be seen in the motor torque.
</p>

<blockquote><p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/ActuatorNoise.png\"/>
</p></blockquote>

<p>
Note, the noise in all components can be easily switched off by setting parameter
enableNoise = false in the globalSeed component.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end ActuatorWithNoise;

    model DrydenContinuousTurbulence
      "Demonstrates how to model wind turbulence for aircraft with the BandLimitedWhiteNoise block (a simple model of vertical Dryden gust speed at low altitudes < 1000 ft)"
      extends Modelica.Icons.Example;

      import Modelica.Constants.pi;

      parameter SI.Velocity V =            140 * 0.5144
        "Airspeed of aircraft (typically 140kts during approach)";
      parameter SI.Velocity sigma = 0.1 *   30 * 0.5144
        "Turbulence intensity (=0.1 * wind at 20 ft, typically 30 kt)";
      parameter SI.Length   L =            600 * 0.3048
        "Scale length (= flight altitude)";

      Blocks.Continuous.TransferFunction Hw(
          b=sigma*sqrt(L/pi/V)*{sqrt(3)*L/V,1},
          a={L^2/V^2,2*L/V,1},
          initType=Blocks.Types.Init.InitialState)
          "Transfer function of vertical turbulence speed according to MIL-F-8785C"
          annotation (Placement(transformation(extent={{-10,0},{10,20}})));
      Blocks.Noise.BandLimitedWhiteNoise whiteNoise(samplePeriod=0.005)
          annotation (Placement(transformation(extent={{-60,0},{-40,20}})));
      constant SI.Velocity unitVelocity = 1 annotation(HideResult=true);
      Blocks.Math.Gain compareToSpeed(k=unitVelocity/V)
          annotation (Placement(transformation(extent={{40,0},{60,20}})));
      inner Blocks.Noise.GlobalSeed globalSeed
          annotation (Placement(transformation(extent={{40,60},{60,80}})));
    equation
      connect(whiteNoise.y, Hw.u) annotation (Line(
          points={{-39,10},{-12,10}}, color={0,0,127}));
      connect(Hw.y, compareToSpeed.u) annotation (Line(
          points={{11,10},{38,10}}, color={0,0,127}));
      annotation (experiment(StopTime=100),
     Documentation(info="<html>
<p>
This example shows how to use the
<a href=\"modelica://Modelica.Blocks.Noise.BandLimitedWhiteNoise\">BandLimitedWhiteNoise</a>
to feed a Dryden continuous turbulence model. This model is used to describe turbulent wind at low altitudes
that varies randomly in space
(see also <a href=\"https://en.wikipedia.org/wiki/Continuous_gusts\">wikipedia</a>).
</p>

<h4>
Turbulence model for vertical gust speed at low altitudes
</h4>

<p>
The turbulence model of the Dryden form is defined by the power spectral density of the vertical turbulent velocity:
</p>

<blockquote><p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/equation-erVWhiWU.png\" alt=\"Phi_w(Omega)=sigma^2*L_w/pi*((1+3*(L_w*Omega)^2)/(1+(L_w*Omega)^2)^2)\"/>
</p></blockquote>

<p>
The spectrum is parametrized with the following parameters:
</p>

<ul>
<li> Lw is the turbulence scale.<br>In low altitudes, it is equal to the flight altitude.</li>
<li> sigma is the turbulence intensity.<br>In low altitudes, it is equal to 1/10 of the
     wind speed at 20 ft altitude, which is 30 kts for medium turbulence.</li>
<li> Omega is the spatial frequency.<br> The turbulence model is thus defined in space and the aircraft experiences turbulence as it flies through the defined wind field.</li>
<li> Omega = s/V will be used to transform the spatial definition into a temporal definition, which can be realized as a state space system.</li>
<li> V is the airspeed of the aircraft.<br>It is approximately 150 kts during the approach (i.e. at low altitudes).</li>
</ul>

<p>
Using spectral factorization and a fixed airspeed V of the aircraft, a concrete forming filter for the vertical turbulence can be found as
</p>

<blockquote><p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/equation-W0zl2Gay.png\" alt=\"H_w(s) = sigma*sqrt(L_w/(pi*V)) * ((1 + sqrt(3)*L_w/V*s) / (1+L_w/V*s)^2)\"/>,
</p></blockquote>

<p>
for which V * (H_w(i Omega/V) * H_w(-i Omega/V) = Phi_w(Omega).
</p>

<h4>
The input to the filter
</h4>

<p>
The input to the filter is white noise with a normal distribution, zero mean, and a power spectral density of 1.
That means, for a sampling time of 1s, it is parameterized with mean=0 and variance=1.
However, in order to account for the change of noise power due to sampling, the noise must be scaled with sqrt(samplePeriod).
This is done automatically in the
<a href=\"modelica://Modelica.Blocks.Noise.BandLimitedWhiteNoise\">BandLimitedWhiteNoise</a> block.
</p>

<h4>Example output</h4>

<blockquote>
<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/DrydenContinuousTurbulence.png\"/>
</p></blockquote>

<h4>
Reference
</h4>

<ol>
<li>Dryden Wind Turbulence model in US military standard
    <a href=\"http://everyspec.com/MIL-SPECS/MIL-SPECS-MIL-F/MIL-F-8785C_5295/\">MIL-F-8785</a>.</li>
</ol>
</html>"));
    end DrydenContinuousTurbulence;

    package Utilities "Library of utility models used in the examples"
      extends Modelica.Icons.UtilitiesPackage;

      block UniformDensity "Calculates the density of a uniform distribution"
        import distribution = Modelica.Math.Distributions.Uniform.density;
        extends Blocks.Icons.Block;

        parameter Real u_min "Lower limit of u";
        parameter Real u_max "Upper limit of u";

        Blocks.Interfaces.RealInput u "Real input signal" annotation (Placement(
                transformation(extent={{-140,-20},{-100,20}})));
        Blocks.Interfaces.RealOutput y
            "Density of the input signal according to the uniform probability density function"
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      equation
        y = distribution(u, u_min, u_max);

        annotation (Icon(graphics={
              Polygon(
                points={{0,94},{-8,72},{8,72},{0,94}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{0,76},{0,-72}}, color={192,192,192}),
              Line(points={{-86,-82},{72,-82}},
                                            color={192,192,192}),
              Polygon(
                points={{92,-82},{70,-74},{70,-90},{92,-82}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
          Line( points={{-70,-75.953},{-66.5,-75.8975},{-63,-75.7852},{-59.5,
                -75.5674},{-56,-75.1631},{-52.5,-74.4442},{-49,-73.2213},{
                -45.5,-71.2318},{-42,-68.1385},{-38.5,-63.5468},{-35,-57.0467},
                {-31.5,-48.2849},{-28,-37.0617},{-24.5,-23.4388},{-21,-7.8318},
                {-17.5,8.9428},{-14,25.695},{-10.5,40.9771},{-7,53.2797},{
                -3.5,61.2739},{0,64.047},{3.5,61.2739},{7,53.2797},{10.5,
                40.9771},{14,25.695},{17.5,8.9428},{21,-7.8318},{24.5,
                -23.4388},{28,-37.0617},{31.5,-48.2849},{35,-57.0467},{38.5,
                -63.5468},{42,-68.1385},{45.5,-71.2318},{49,-73.2213},{52.5,
                -74.4442},{56,-75.1631},{59.5,-75.5674},{63,-75.7852},{66.5,
                -75.8975},{70,-75.953}},
                smooth=Smooth.Bezier)}), Documentation(info="<html>
<p>
This block determines the probability density y of a uniform distribution for the given input signal u
(for details of this density function see
<a href=\"modelica://Modelica.Math.Distributions.Uniform.density\">Math.Distributions.Uniform.density</a>).
</p>

<p>
This block is demonstrated in the example
<a href=\"modelica://Modelica.Blocks.Examples.Noise.Densities\">Examples.Noise.Densities</a> .
</p>
</html>",       revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
      end UniformDensity;

      block NormalDensity "Calculates the density of a normal distribution"
        import distribution = Modelica.Math.Distributions.Normal.density;
        extends Blocks.Icons.Block;

        parameter Real mu "Expectation (mean) value of the normal distribution";
        parameter Real sigma "Standard deviation of the normal distribution";

        Blocks.Interfaces.RealInput u "Real input signal" annotation (Placement(
                transformation(extent={{-140,-20},{-100,20}})));
        Blocks.Interfaces.RealOutput y
            "Density of the input signal according to the normal probability density function"
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      equation
        y = distribution(u, mu, sigma);

        annotation (Icon(graphics={
              Polygon(
                points={{0,94},{-8,72},{8,72},{0,94}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{0,76},{0,-72}}, color={192,192,192}),
              Line(points={{-86,-82},{72,-82}},
                                            color={192,192,192}),
              Polygon(
                points={{92,-82},{70,-74},{70,-90},{92,-82}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
          Line( points={{-70,-75.953},{-66.5,-75.8975},{-63,-75.7852},{-59.5,
                -75.5674},{-56,-75.1631},{-52.5,-74.4442},{-49,-73.2213},{
                -45.5,-71.2318},{-42,-68.1385},{-38.5,-63.5468},{-35,-57.0467},
                {-31.5,-48.2849},{-28,-37.0617},{-24.5,-23.4388},{-21,-7.8318},
                {-17.5,8.9428},{-14,25.695},{-10.5,40.9771},{-7,53.2797},{
                -3.5,61.2739},{0,64.047},{3.5,61.2739},{7,53.2797},{10.5,
                40.9771},{14,25.695},{17.5,8.9428},{21,-7.8318},{24.5,
                -23.4388},{28,-37.0617},{31.5,-48.2849},{35,-57.0467},{38.5,
                -63.5468},{42,-68.1385},{45.5,-71.2318},{49,-73.2213},{52.5,
                -74.4442},{56,-75.1631},{59.5,-75.5674},{63,-75.7852},{66.5,
                -75.8975},{70,-75.953}},
                smooth=Smooth.Bezier)}), Documentation(info="<html>
<p>
This block determines the probability density y of a normal distribution for the given input signal u
(for details of this density function see
<a href=\"modelica://Modelica.Math.Distributions.Normal.density\">Math.Distributions.Normal.density</a>).
</p>

<p>
This block is demonstrated in the example
<a href=\"modelica://Modelica.Blocks.Examples.Noise.Densities\">Examples.Noise.Densities</a> .
</p>
</html>",       revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
      end NormalDensity;

      block WeibullDensity "Calculates the density of a Weibull distribution"
        import distribution = Modelica.Math.Distributions.Weibull.density;
        extends Blocks.Icons.Block;

        parameter Real lambda(min=0)
          "Scale parameter of the Weibull distribution";
        parameter Real k(min=0) "Shape parameter of the Weibull distribution";

        Blocks.Interfaces.RealInput u "Real input signal" annotation (Placement(
                transformation(extent={{-140,-20},{-100,20}})));
        Blocks.Interfaces.RealOutput y
            "Density of the input signal according to the Weibull probability density function"
            annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      equation
        y = distribution(u, lambda, k);

        annotation (Icon(graphics={
              Polygon(
                points={{0,94},{-8,72},{8,72},{0,94}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{0,76},{0,-72}}, color={192,192,192}),
              Line(points={{-86,-82},{72,-82}},
                                            color={192,192,192}),
              Polygon(
                points={{92,-82},{70,-74},{70,-90},{92,-82}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
          Line( points={{-70,-75.953},{-66.5,-75.8975},{-63,-75.7852},{-59.5,
                -75.5674},{-56,-75.1631},{-52.5,-74.4442},{-49,-73.2213},{
                -45.5,-71.2318},{-42,-68.1385},{-38.5,-63.5468},{-35,-57.0467},
                {-31.5,-48.2849},{-28,-37.0617},{-24.5,-23.4388},{-21,-7.8318},
                {-17.5,8.9428},{-14,25.695},{-10.5,40.9771},{-7,53.2797},{
                -3.5,61.2739},{0,64.047},{3.5,61.2739},{7,53.2797},{10.5,
                40.9771},{14,25.695},{17.5,8.9428},{21,-7.8318},{24.5,
                -23.4388},{28,-37.0617},{31.5,-48.2849},{35,-57.0467},{38.5,
                -63.5468},{42,-68.1385},{45.5,-71.2318},{49,-73.2213},{52.5,
                -74.4442},{56,-75.1631},{59.5,-75.5674},{63,-75.7852},{66.5,
                -75.8975},{70,-75.953}},
                smooth=Smooth.Bezier)}), Documentation(info="<html>
<p>
This block determines the probability density y of a Weibull distribution for the given input signal u
(for details of this density function see
<a href=\"modelica://Modelica.Math.Distributions.Weibull.density\">Math.Distributions.Weibull.density</a>).
</p>

<p>
This block is demonstrated in the example
<a href=\"modelica://Modelica.Blocks.Examples.Noise.Densities\">Examples.Noise.Densities</a> .
</p>
</html>",       revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
      end WeibullDensity;

      block ImpureRandom
        "Block generating random numbers with the impure random number generator"
        extends Blocks.Interfaces.SO;

        parameter SI.Period samplePeriod
          "Sample period for random number generation";

        protected
         outer Blocks.Noise.GlobalSeed globalSeed;

      equation
         when {initial(), sample(samplePeriod,samplePeriod)} then
            y = Modelica.Math.Random.Utilities.impureRandom(globalSeed.id_impure);
         end when;
        annotation (Documentation(info="<html>
<p>
This block demonstrates how to implement a block using the impure
random number generator. This block is used in the example
<a href=\"modelica://Modelica.Blocks.Examples.Noise.ImpureGenerator\">Examples.Noise.ImpureGenerator</a>.
</p>
</html>",       revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
      end ImpureRandom;

      package Parts "Parts for use in the ActuatorWithNoise examples"
        extends Modelica.Icons.Package;

        model MotorWithCurrentControl
          "Synchronous machine with current controller and measurement noise"
          extends Modelica.Electrical.Machines.Icons.TransientMachine;
          constant Integer m=3 "Number of phases";
          parameter SI.Voltage VNominal=100
            "Nominal RMS voltage per phase";
          parameter SI.Frequency fNominal=50 "Nominal frequency";
          parameter SI.Frequency f=50 "Actual frequency";
          parameter SI.Time tRamp=1 "Frequency ramp";
          parameter SI.Torque TLoad=181.4 "Nominal load torque";
          parameter SI.Time tStep=1.2 "Time of load torque step";
          parameter SI.Inertia JLoad=0.29 "Load's moment of inertia";

          Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet
            smpm(
            p=smpmData.p,
            fsNominal=smpmData.fsNominal,
            Rs=smpmData.Rs,
            TsRef=smpmData.TsRef,
            Lszero=smpmData.Lszero,
            Lssigma=smpmData.Lssigma,
            Jr=smpmData.Jr,    Js=smpmData.Js,
            frictionParameters=smpmData.frictionParameters,
            wMechanical(fixed=true),
            statorCoreParameters=smpmData.statorCoreParameters,
            strayLoadParameters=smpmData.strayLoadParameters,
            VsOpenCircuit=smpmData.VsOpenCircuit,
            Lmd=smpmData.Lmd,
            Lmq=smpmData.Lmq,
            useDamperCage=smpmData.useDamperCage,
            Lrsigmad=smpmData.Lrsigmad,
            Lrsigmaq=smpmData.Lrsigmaq,
            Rrd=smpmData.Rrd,
            Rrq=smpmData.Rrq,
            TrRef=smpmData.TrRef,
            permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
            phiMechanical(fixed=true),
            TsOperational=293.15,
            alpha20s=smpmData.alpha20s,
            TrOperational=293.15,
            alpha20r=smpmData.alpha20r)
            annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
          Modelica.Electrical.Polyphase.Sources.SignalCurrent signalCurrent(final m=m)
            annotation (Placement(transformation(
                origin={-10,50},
                extent={{-10,10},{10,-10}},
                rotation=270)));
          Modelica.Electrical.Polyphase.Basic.Star star(final m=m)
            annotation (Placement(transformation(extent={{-10,80},{-30,100}})));
          Modelica.Electrical.Analog.Basic.Ground ground
            annotation (Placement(transformation(
                origin={-50,90},
                extent={{-10,-10},{10,10}},
                rotation=270)));
          Modelica.Electrical.Machines.Utilities.DQToThreePhase dqToThreePhase(
              p=smpm.p)
            annotation (Placement(transformation(extent={{-50,40},{-30,60}})));
          Modelica.Electrical.Polyphase.Basic.Star starM(final m=m) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={-60,-10})));
          Modelica.Electrical.Analog.Basic.Ground groundM
            annotation (Placement(transformation(
                origin={-80,-28},
                extent={{-10,-10},{10,10}},
                rotation=270)));
          Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(
              terminalConnection="Y") annotation (Placement(transformation(extent={{-20,-30},
                    {0,-10}})));
          Modelica.Electrical.Machines.Sensors.RotorDisplacementAngle rotorDisplacementAngle(p=smpm.p)
            annotation (Placement(transformation(
                origin={20,-40},
                extent={{-10,10},{10,-10}},
                rotation=270)));
          Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
              Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={10,0})));
          Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor annotation (
              Placement(transformation(
                extent={{10,10},{-10,-10}},
                rotation=180,
                origin={50,-40})));
          Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
              Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={30,0})));
          Modelica.Electrical.Machines.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor
            annotation (Placement(transformation(
                extent={{-10,10},{10,-10}},
                rotation=180,
                origin={-30,-10})));
          Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentQuasiRMSSensor
            annotation (Placement(transformation(
                origin={-10,0},
                extent={{-10,-10},{10,10}},
                rotation=270)));
          Modelica.Mechanics.Rotational.Components.Inertia inertiaLoad(J=0.29)
            annotation (Placement(transformation(extent={{70,-50},{90,-30}})));
          parameter
            Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData
            smpmData(useDamperCage=false) "Data for motor"
            annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));
          Blocks.Sources.Constant id(k=0) annotation (Placement(transformation(
                    extent={{-90,60},{-70,80}})));
          Blocks.Interfaces.RealInput iq_rms1 annotation (Placement(
                  transformation(extent={{-140,40},{-100,80}}),
                  iconTransformation(extent={{-140,40},{-100,80}})));
          Modelica.Mechanics.Rotational.Interfaces.Flange_b flange
            "Right flange of shaft"
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));
          Blocks.Interfaces.RealOutput phi(unit="rad")
              "Absolute angle of flange as output signal" annotation (Placement(
                  transformation(extent={{-10,-10},{10,10}}, origin={110,80}),
                  iconTransformation(extent={{40,70},{60,90}})));
          output Real phi_motor(unit="rad", displayUnit="deg")=angleSensor.phi
            "Rotational position";
          output Real w(unit="rad/s")=speedSensor.w "Rotational speed";
          Blocks.Math.Add addNoise
              annotation (Placement(transformation(extent={{60,70},{80,90}})));
          .Blocks.Noise.UniformNoise uniformNoise(
              samplePeriod=1/200,
              y_min=-0.01,
              y_max=0.01)
              annotation (Placement(transformation(extent={{26,76},{46,96}})));
        equation
          connect(star.pin_n, ground.p) annotation (Line(points={{-30,90},{-40,90}}, color={0,0,255}));
          connect(rotorDisplacementAngle.plug_n, smpm.plug_sn) annotation (Line(
                points={{26,-30},{26,-20},{-16,-20},{-16,-30}}, color={0,0,255}));
          connect(rotorDisplacementAngle.plug_p, smpm.plug_sp) annotation (Line(
                points={{14,-30},{6,-30},{-4,-30}}, color={0,0,255}));
          connect(terminalBox.plug_sn, smpm.plug_sn) annotation (Line(
              points={{-16,-26},{-16,-30}}, color={0,0,255}));
          connect(terminalBox.plug_sp, smpm.plug_sp) annotation (Line(
              points={{-4,-26},{-4,-30}}, color={0,0,255}));
          connect(smpm.flange, rotorDisplacementAngle.flange) annotation (Line(
              points={{0,-40},{6,-40},{10,-40}}));
          connect(signalCurrent.plug_p, star.plug_p) annotation (Line(
              points={{-10,60},{-10,90}}, color={0,0,255}));
          connect(angleSensor.flange, rotorDisplacementAngle.flange) annotation (Line(
              points={{10,-10},{10,-40}}));
          connect(angleSensor.phi, dqToThreePhase.phi) annotation (Line(points={{10,11},
                  {10,30},{-40,30},{-40,38}},          color={0,0,127}));
          connect(groundM.p, terminalBox.starpoint) annotation (Line(
              points={{-70,-28},{-20,-28},{-20,-24}}, color={0,0,255}));
          connect(smpm.flange, torqueSensor.flange_a) annotation (Line(
              points={{0,-40},{40,-40}}));
          connect(voltageQuasiRMSSensor.plug_p, terminalBox.plugSupply) annotation (
              Line(
              points={{-20,-10},{-10,-10},{-10,-24}}, color={0,0,255}));
          connect(starM.plug_p, voltageQuasiRMSSensor.plug_n) annotation (Line(
              points={{-50,-10},{-40,-10}}, color={0,0,255}));
          connect(starM.pin_n, groundM.p) annotation (Line(
              points={{-70,-10},{-70,-28}}, color={0,0,255}));
          connect(dqToThreePhase.y, signalCurrent.i) annotation (Line(points={{
                  -29,50},{-22,50},{-22,50}}, color={0,0,127}));
          connect(speedSensor.flange, smpm.flange) annotation (Line(
              points={{30,-10},{30,-40},{0,-40}}));
          connect(torqueSensor.flange_b, inertiaLoad.flange_a) annotation (Line(
              points={{60,-40},{60,-40},{70,-40}}));
          connect(signalCurrent.plug_n, currentQuasiRMSSensor.plug_p) annotation (
             Line(
              points={{-10,40},{-10,10}}, color={0,0,255}));
          connect(currentQuasiRMSSensor.plug_n, voltageQuasiRMSSensor.plug_p)
            annotation (Line(
              points={{-10,-10},{-20,-10}}, color={0,0,255}));
          connect(inertiaLoad.flange_b, flange) annotation (Line(
              points={{90,-40},{90,-40},{90,0},{100,0}}));
          connect(angleSensor.phi, addNoise.u2) annotation (Line(
              points={{10,11},{10,30},{50,30},{50,74},{58,74}}, color={0,0,127}));
          connect(addNoise.y, phi) annotation (Line(
              points={{81,80},{110,80}}, color={0,0,127}));
          connect(uniformNoise.y, addNoise.u1) annotation (Line(
              points={{47,86},{58,86}}, color={0,0,127}));
          connect(id.y, dqToThreePhase.d) annotation (Line(points={{-69,70},{-60,
                  70},{-60,56},{-52,56}}, color={0,0,127}));
          connect(iq_rms1, dqToThreePhase.q) annotation (Line(points={{-120,60},
                  {-100,60},{-100,44},{-52,44}}, color={0,0,127}));
          annotation (
            Documentation(info="<html>
<p>
A synchronous machine with permanent magnets, current controller and
measurement noise of &plusmn;0.01 rad accelerates a quadratic speed dependent load from standstill.
The rms values of d- and q-current in rotor fixed coordinate system are converted to three-phase currents,
and fed to the machine. The result shows that the torque is influenced by the q-current,
whereas the stator voltage is influenced by the d-current.
</p>

<p>
Default machine parameters of model
<a href=\"modelica://Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet\">SM_PermanentMagnet</a>
are used.
</p>

<p>
This motor is used in the
<a href=\"modelica://Modelica.Blocks.Examples.Noise.ActuatorWithNoise\">Examples.Noise.ActuatorWithNoise</a>
actuator example
</p>
</html>",         revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"), Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                    100}}), graphics={Rectangle(
                  extent={{40,50},{-100,100}},
                  fillColor={255,170,85},
                  fillPattern=FillPattern.Solid,
                  pattern=LinePattern.None), Text(
                extent={{-150,150},{150,110}},
                textString="%name",
                textColor={0,0,255})}));
        end MotorWithCurrentControl;

        model Controller "Simple position controller for actuator"
          extends Blocks.Icons.Block;

          Blocks.Continuous.PI speed_PI(
              k=10,
              T=5e-2,
              initType=Blocks.Types.Init.InitialOutput)
              annotation (Placement(transformation(extent={{38,-10},{58,10}})));
          Blocks.Math.Feedback speedFeedback
              annotation (Placement(transformation(extent={{10,-10},{30,10}})));
          Blocks.Continuous.Derivative positionToSpeed(initType=Blocks.Types.Init.InitialOutput,
                T=0.01) annotation (Placement(transformation(extent={{-60,-70},
                      {-40,-50}})));
          Blocks.Interfaces.RealInput positionMeasured
              "Position signal of motor" annotation (Placement(transformation(
                    extent={{-140,-80},{-100,-40}})));
          Blocks.Interfaces.RealInput positionReference "Reference position"
              annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
          Blocks.Interfaces.RealOutput y1 "Connector of Real output signal"
              annotation (Placement(transformation(extent={{100,-10},{120,10}})));
          Blocks.Continuous.PI position_PI(
              T=5e-1,
              k=3,
              initType=Blocks.Types.Init.InitialState) annotation (Placement(
                  transformation(extent={{-60,50},{-40,70}})));
          Blocks.Math.Feedback positionFeedback annotation (Placement(
                  transformation(extent={{-90,50},{-70,70}})));
          Blocks.Continuous.FirstOrder busdelay(T=1e-3, initType=Blocks.Types.Init.InitialOutput)
              annotation (Placement(transformation(extent={{68,-10},{88,10}})));
        equation
          connect(speedFeedback.y, speed_PI.u) annotation (Line(
              points={{29,0},{36,0}}, color={0,0,127}));
          connect(positionFeedback.u2, positionToSpeed.u) annotation (Line(
              points={{-80,52},{-80,-60},{-62,-60}}, color={0,0,127}));
          connect(positionReference, positionFeedback.u1) annotation (Line(
              points={{-120,60},{-88,60}}, color={0,0,127}));
          connect(positionFeedback.y, position_PI.u) annotation (Line(
              points={{-71,60},{-62,60}}, color={0,0,127}));
          connect(position_PI.y, speedFeedback.u1) annotation (Line(
              points={{-39,60},{0,60},{0,0},{12,0}}, color={0,0,127}));
          connect(speed_PI.y, busdelay.u) annotation (Line(
              points={{59,0},{66,0}}, color={0,0,127}));
          connect(y1, busdelay.y) annotation (Line(
              points={{110,0},{89,0}}, color={0,0,127}));
          connect(positionMeasured, positionToSpeed.u) annotation (Line(
              points={{-120,-60},{-62,-60}}, color={0,0,127}));
          connect(positionToSpeed.y, speedFeedback.u2) annotation (Line(
              points={{-39,-60},{20,-60},{20,-8}}, color={0,0,127}));
          annotation (Icon(coordinateSystem(
                  preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
                Text(
                  extent={{-40,50},{40,-30}},
                  textColor={0,0,255},
                  textString="PI")}),
            Documentation(revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>",         info="<html>
<p>
A simple position controller for a drive system.
This controller is used in the
<a href=\"modelica://Modelica.Blocks.Examples.Noise.ActuatorWithNoise\">Examples.Noise.ActuatorWithNoise</a>
actuator example
</p>
</html>"));
        end Controller;
      annotation (Documentation(revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>",       info="<html>
<p>
Parts used in the
<a href=\"modelica://Modelica.Blocks.Examples.Noise.ActuatorWithNoise\">Examples.Noise.ActuatorWithNoise</a>
actuator example
</p>
</html>"));
      end Parts;
    annotation (Documentation(info="<html>
<p>
This package contains utility models that are used for the examples.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end Utilities;
  annotation (Documentation(info="<html>
<p>
This package contains various example models that demonstrates how
to utilize the blocks from sublibrary
<a href=\"modelica://Modelica.Blocks.Noise\">Blocks.Noise</a>.
</p>
</html>",   revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
  end Noise;

  package BusUsage_Utilities
    "Utility models and connectors for example Modelica.Blocks.Examples.BusUsage"
    extends Modelica.Icons.UtilitiesPackage;
    package Interfaces "Interfaces specialised for this example"
      extends Modelica.Icons.InterfacesPackage;

      expandable connector ControlBus
        "Control bus that is adapted to the signals connected to it"
        extends Modelica.Icons.SignalBus;

        SI.AngularVelocity realSignal1 "First Real signal (angular velocity)"
          annotation (HideResult=false);
        SI.Velocity realSignal2 "Second Real signal"
          annotation (HideResult=false);
        Integer integerSignal "Integer signal" annotation (HideResult=false);
        Boolean booleanSignal "Boolean signal" annotation (HideResult=false);
        SubControlBus subControlBus "Combined signal"
          annotation (HideResult=false);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Rectangle(
                        extent={{-20,2},{22,-2}},
                        lineColor={255,204,51},
                        lineThickness=0.5)}), Documentation(info="<html>
<p>
This connector defines the \"expandable connector\" ControlBus that
is used as bus in the
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a> example.
Note, this connector contains \"default\" signals that might be utilized
in a connection (the input/output causalities of the signals
are determined from the connections to this bus).
</p>
</html>"));

      end ControlBus;

      expandable connector SubControlBus
        "Sub-control bus that is adapted to the signals connected to it"
        extends Modelica.Icons.SignalSubBus;
        Real myRealSignal annotation (HideResult=false);
        Boolean myBooleanSignal annotation (HideResult=false);
        annotation (
          defaultComponentPrefixes="protected",
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={Rectangle(
                        extent={{-20,2},{22,-2}},
                        lineColor={255,204,51},
                        lineThickness=0.5)}),
          Documentation(info="<html>
<p>
This connector defines the \"expandable connector\" SubControlBus that
is used as sub-bus in the
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a> example.
Note, this is an expandable connector which has a \"default\" set of
signals (the input/output causalities of the signals are
determined from the connections to this bus).
</p>
</html>"));

      end SubControlBus;

      annotation (Documentation(info="<html>
<p>
This package contains the bus definitions needed for the
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a> example.
</p>
</html>"));
    end Interfaces;

    model Part "Component with sub-control bus"

      Interfaces.SubControlBus subControlBus annotation (Placement(
            transformation(
            origin={100,0},
            extent={{-20,-20},{20,20}},
            rotation=270)));
      Sources.RealExpression realExpression(y=time) annotation (Placement(
            transformation(extent={{-6,0},{20,20}})));
      Sources.BooleanExpression booleanExpression(y=time >= 0.5) annotation (
          Placement(transformation(extent={{-6,-30},{20,-10}})));
    equation
      connect(realExpression.y, subControlBus.myRealSignal) annotation (Line(
          points={{21.3,10},{88,10},{88,6},{98,6},{98,0},{100,0}}, color={0,0,127}));
      connect(booleanExpression.y, subControlBus.myBooleanSignal) annotation (
          Line(
          points={{21.3,-20},{60,-20},{60,0},{100,0}}, color={255,0,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={Rectangle(
              extent={{-100,60},{100,-60}},
              fillColor={159,159,223},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}), Text(
              extent={{-106,124},{114,68}},
              textString="%name",
              textColor={0,0,255})}), Documentation(info="<html>
<p>
This model is used to demonstrate the bus usage in example
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a>.
</p>
</html>"));
    end Part;

    annotation (Documentation(info="<html>
<p>
This package contains utility models and bus definitions needed for the
<a href=\"modelica://Modelica.Blocks.Examples.BusUsage\">BusUsage</a> example.
</p>
</html>"));
  end BusUsage_Utilities;
  annotation (Documentation(info="<html>
<p>
This package contains example models to demonstrate the
usage of package blocks.
</p>
</html>"));
end Examples;

  package Continuous "Library of continuous control blocks with internal states"

    import Blocks.Interfaces;

    extends Modelica.Icons.Package;

    block Integrator "Output the integral of the input signal with optional reset"
      import Blocks.Types.Init;
      parameter Real k(unit="1")=1 "Integrator gain";
      parameter Boolean use_reset = false "= true, if reset port enabled"
        annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
      parameter Boolean use_set = false "= true, if set port enabled and used as reinitialization value when reset"
        annotation(Dialog(enable=use_reset), Evaluate=true, HideResult=true, choices(checkBox=true));

      /* InitialState is the default, because it was the default in Modelica 2.2
     and therefore this setting is backward compatible
  */
      parameter Init initType=Init.InitialState
        "Type of initialization (1: no init, 2: steady state, 3,4: initial output)" annotation(Evaluate=true,
          Dialog(group="Initialization"));
      parameter Real y_start=0 "Initial or guess value of output (= state)"
        annotation (Dialog(group="Initialization"));
      extends Interfaces.SISO(y(start=y_start));
      Blocks.Interfaces.BooleanInput reset if use_reset
        "Optional connector of reset signal" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={60,-120})));
      Blocks.Interfaces.RealInput set if use_reset and use_set
        "Optional connector of set signal" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,120})));
    protected
      Blocks.Interfaces.BooleanOutput local_reset annotation (HideResult=true);
      Blocks.Interfaces.RealOutput local_set annotation (HideResult=true);

    initial equation
      if initType == Init.SteadyState then
         der(y) = 0;
      elseif initType == Init.InitialState or
             initType == Init.InitialOutput then
        y = y_start;
      end if;
    equation
      if use_reset then
        connect(reset, local_reset);
        if use_set then
          connect(set, local_set);
        else
          local_set = y_start;
        end if;
        when local_reset then
          reinit(y, local_set);
        end when;
      else
        local_reset = false;
        local_set = 0;
      end if;
      der(y) = k*u;
      annotation (
        Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as
<em>integral</em> of the input <strong>u</strong> multiplied with
the gain <em>k</em>:
</p>
<blockquote><pre>
    k
y = - u
    s
</pre></blockquote>

<p>
It might be difficult to initialize the integrator in steady state.
This is discussed in the description of package
<a href=\"modelica://Modelica.Blocks.Continuous#info\">Continuous</a>.
</p>

<p>
If the <em>reset</em> port is enabled, then the output <strong>y</strong> is reset to <em>set</em>
or to <em>y_start</em> (if the <em>set</em> port is not enabled), whenever the <em>reset</em>
port has a rising edge.
</p>
</html>"),   Icon(coordinateSystem(
              preserveAspectRatio=true,
              extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
              Line(
                points={{-80.0,78.0},{-80.0,-90.0}},
                color={192,192,192}),
              Polygon(
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid,
                points={{-80.0,90.0},{-88.0,68.0},{-72.0,68.0},{-80.0,90.0}}),
              Line(
                points={{-90.0,-80.0},{82.0,-80.0}},
                color={192,192,192}),
              Polygon(
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid,
                points={{90.0,-80.0},{68.0,-72.0},{68.0,-88.0},{90.0,-80.0}}),
              Text(
                textColor={192,192,192},
                extent={{0.0,-70.0},{60.0,-10.0}},
                textString="I"),
              Text(
                extent={{-150.0,-150.0},{150.0,-110.0}},
                textString="k=%k"),
              Line(
                points=DynamicSelect({{-80.0,-80.0},{80.0,80.0}}, if use_reset then {{-80.0,-80.0},{60.0,60.0},{60.0,-80.0},{80.0,-60.0}} else {{-80.0,-80.0},{80.0,80.0}}),
                color={0,0,127}),
              Line(
                visible=use_reset,
                points={{60,-100},{60,-80}},
                color={255,0,255},
                pattern=LinePattern.Dot),
              Text(
                visible=use_reset,
                extent={{-28,-62},{94,-86}},
                textString="reset")}));
    end Integrator;

    block LimIntegrator "Integrator with limited value of the output and optional reset"
      import Blocks.Types.Init;
      parameter Real k(unit="1")=1 "Integrator gain";
      parameter Real outMax(start=1) "Upper limit of output";
      parameter Real outMin=-outMax "Lower limit of output";
      parameter Boolean use_reset = false "= true, if reset port enabled"
        annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
      parameter Boolean use_set = false "= true, if set port enabled and used as reinitialization value when reset"
        annotation(Dialog(enable=use_reset), Evaluate=true, HideResult=true, choices(checkBox=true));
      parameter Init initType=Init.InitialState
        "Type of initialization (1: no init, 2: steady state, 3/4: initial output)"
        annotation(Evaluate=true, Dialog(group="Initialization"));
      parameter Boolean limitsAtInit = true
        "= false, if limits are ignored during initialization (i.e., der(y)=k*u)"
        annotation(Evaluate=true, Dialog(group="Initialization"));
      parameter Real y_start=0
        "Initial or guess value of output (must be in the limits outMin .. outMax)"
        annotation (Dialog(group="Initialization"));
      parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
        annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
      extends Interfaces.SISO(y(start=y_start));
      Blocks.Interfaces.BooleanInput reset if use_reset
        "Optional connector of reset signal" annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={60,-120})));
      Blocks.Interfaces.RealInput set if use_reset and use_set
        "Optional connector of set signal" annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,120})));
    protected
      Blocks.Interfaces.BooleanOutput local_reset annotation (HideResult=true);
      Blocks.Interfaces.RealOutput local_set annotation (HideResult=true);

    initial equation
      if initType == Init.SteadyState then
         der(y) = 0;
      elseif initType == Init.InitialState or
             initType == Init.InitialOutput then
        y = y_start;
      end if;
    equation
      if use_reset then
        connect(reset, local_reset);
        if use_set then
          connect(set, local_set);
        else
          local_set = y_start;
        end if;
        when local_reset then
          reinit(y, if local_set < outMin then outMin elseif local_set > outMax then outMax else local_set);
        end when;
      else
        local_reset = false;
        local_set = 0;
      end if;
      if initial() and not limitsAtInit then
         der(y) = k*u;
         assert(y >= outMin - 0.001*abs(outMax-outMin) and y <= outMax + 0.001*abs(outMax-outMin),
              "LimIntegrator: During initialization the limits have been ignored.\n"
            + "However, the result is that the output y is not within the required limits:\n"
            + "  y = " + String(y) + ", outMin = " + String(outMin) + ", outMax = " + String(outMax));
      elseif strict then
         der(y) = noEvent(if y < outMin and k*u < 0 or y > outMax and k*u > 0 then 0 else k*u);
      else
         der(y) = if y < outMin and k*u < 0 or y > outMax and k*u > 0 then 0 else k*u;
      end if;
      annotation (
        Documentation(info="<html>
<p>
This blocks computes <strong>y</strong> as <em>integral</em>
of the input <strong>u</strong> multiplied with the gain <em>k</em>. If the
integral reaches a given upper or lower <em>limit</em> and the
input will drive the integral outside of this bound, the
integration is halted and only restarted if the input drives
the integral away from the bounds.
</p>

<p>
It might be difficult to initialize the integrator in steady state.
This is discussed in the description of package
<a href=\"modelica://Modelica.Blocks.Continuous#info\">Continuous</a>.
</p>

<p>
If parameter <strong>limitsAtInit</strong> = <strong>false</strong>, the limits of the
integrator are removed from the initialization problem which
leads to a much simpler equation system. After initialization has been
performed, it is checked via an assert whether the output is in the
defined limits. For backward compatibility reasons
<strong>limitsAtInit</strong> = <strong>true</strong>. In most cases it is best
to use <strong>limitsAtInit</strong> = <strong>false</strong>.
</p>
<p>
If the <em>reset</em> port is enabled, then the output <strong>y</strong> is reset to <em>set</em>
or to <em>y_start</em> (if the <em>set</em> port is not enabled), whenever the <em>reset</em>
port has a rising edge.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,78},{-80,-90}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-80},{82,-80}}, color={192,192,192}),
            Polygon(
              points={{90,-80},{68,-72},{68,-88},{90,-80}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points=DynamicSelect({{-80,-80},{20,20},{80,20}}, if use_reset then {{-80,-80},{20,20},{60,20},{60,-80},{80,-60}} else {{-80,-80},{20,20},{80,20}}),
              color={0,0,127}),
            Text(
              extent={{0,-10},{60,-70}},
              textColor={192,192,192},
              textString="I"),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="k=%k"),
            Line(
              visible=strict,
              points=DynamicSelect({{20,20},{80,20}}, if use_reset then {{20,20},{60,20}} else {{20,20},{80,20}}),
              color={255,0,0}),
            Line(
              visible=use_reset,
              points={{60,-100},{60,-80}},
              color={255,0,255},
              pattern=LinePattern.Dot),
            Text(
              visible=use_reset,
              extent={{-28,-62},{94,-86}},
              textString="reset")}));
    end LimIntegrator;

    block Derivative "Approximated derivative block"
      import Blocks.Types.Init;
      parameter Real k(unit="1")=1 "Gains";
      parameter SI.Time T(min=Modelica.Constants.small) = 0.01
        "Time constants (T>0 required; T=0 is ideal derivative block)";
      parameter Init initType=Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
                                                                                        annotation(Evaluate=true,
          Dialog(group="Initialization"));
      parameter Real x_start=0 "Initial or guess value of state"
        annotation (Dialog(group="Initialization"));
      parameter Real y_start=0 "Initial value of output (= state)"
        annotation(Dialog(enable=initType == Init.InitialOutput, group=
              "Initialization"));
      extends Interfaces.SISO;

      output Real x(start=x_start) "State of block";

    protected
      parameter Boolean zeroGain = abs(k) < Modelica.Constants.eps;
    initial equation
      if initType == Init.SteadyState then
        der(x) = 0;
      elseif initType == Init.InitialState then
        x = x_start;
      elseif initType == Init.InitialOutput then
        if zeroGain then
           x = u;
        else
           y = y_start;
        end if;
      end if;
    equation
      der(x) = if zeroGain then 0 else (u - x)/T;
      y = if zeroGain then 0 else (k/T)*(u - x);
      annotation (
        Documentation(info="<html>
<p>
This blocks defines the transfer function between the
input u and the output y
as <em>approximated derivative</em>:
</p>
<blockquote><pre>
        k * s
y = ------------ * u
       T * s + 1
</pre></blockquote>
<p>
If you would like to be able to change easily between different
transfer functions (FirstOrder, SecondOrder, ... ) by changing
parameters, use the general block <strong>TransferFunction</strong> instead
and model a derivative block with parameters<br>
b = {k,0}, a = {T, 1}.
</p>

<p>
If k=0, the block reduces to y=0.
</p>
</html>"),   Icon(
        coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Line(points={{-80.0,78.0},{-80.0,-90.0}},
          color={192,192,192}),
      Polygon(lineColor={192,192,192},
        fillColor={192,192,192},
        fillPattern=FillPattern.Solid,
        points={{-80.0,90.0},{-88.0,68.0},{-72.0,68.0},{-80.0,90.0}}),
      Line(points={{-90.0,-80.0},{82.0,-80.0}},
        color={192,192,192}),
      Polygon(lineColor={192,192,192},
        fillColor={192,192,192},
        fillPattern=FillPattern.Solid,
        points={{90.0,-80.0},{68.0,-72.0},{68.0,-88.0},{90.0,-80.0}}),
      Line(origin = {-24.667,-27.333},
        points = {{-55.333,87.333},{-19.333,-40.667},{86.667,-52.667}},
        color = {0,0,127},
        smooth = Smooth.Bezier),
      Text(textColor={192,192,192},
        extent={{-30.0,14.0},{86.0,60.0}},
        textString="DT1"),
      Text(extent={{-150.0,-150.0},{150.0,-110.0}},
        textString="k=%k")}));
    end Derivative;

    block FirstOrder "First order transfer function block (= 1 pole)"
      import Blocks.Types.Init;
      parameter Real k(unit="1")=1 "Gain";
      parameter SI.Time T(start=1) "Time Constant";
      parameter Init initType=Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3/4: initial output)" annotation(Evaluate=true,
          Dialog(group="Initialization"));
      parameter Real y_start=0 "Initial or guess value of output (= state)"
        annotation (Dialog(group="Initialization"));

      extends Interfaces.SISO(y(start=y_start));

    initial equation
      if initType == Init.SteadyState then
        der(y) = 0;
      elseif initType == Init.InitialState or initType == Init.InitialOutput then
        y = y_start;
      end if;
    equation
      der(y) = (k*u - y)/T;
      annotation (
        Documentation(info="<html>
<p>
This blocks defines the transfer function between the input u
and the output y as <em>first order</em> system:
</p>
<blockquote><pre>
          k
y = ------------ * u
       T * s + 1
</pre></blockquote>
<p>
If you would like to be able to change easily between different
transfer functions (FirstOrder, SecondOrder, ... ) by changing
parameters, use the general block <strong>TransferFunction</strong> instead
and model a first order SISO system with parameters<br>
b = {k}, a = {T, 1}.
</p>
<blockquote><pre>
Example:
   parameter: k = 0.3, T = 0.4
   results in:
             0.3
      y = ----------- * u
          0.4 s + 1.0
</pre></blockquote>

</html>"),   Icon(
      coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
        graphics={
      Line(points={{-80.0,78.0},{-80.0,-90.0}},
        color={192,192,192}),
      Polygon(lineColor={192,192,192},
        fillColor={192,192,192},
        fillPattern=FillPattern.Solid,
        points={{-80.0,90.0},{-88.0,68.0},{-72.0,68.0},{-80.0,90.0}}),
      Line(points={{-90.0,-80.0},{82.0,-80.0}},
        color={192,192,192}),
      Polygon(lineColor={192,192,192},
        fillColor={192,192,192},
        fillPattern=FillPattern.Solid,
        points={{90.0,-80.0},{68.0,-72.0},{68.0,-88.0},{90.0,-80.0}}),
      Line(origin = {-26.667,6.667},
          points = {{106.667,43.333},{-13.333,29.333},{-53.333,-86.667}},
          color = {0,0,127},
          smooth = Smooth.Bezier),
      Text(textColor={192,192,192},
        extent={{0.0,-60.0},{60.0,0.0}},
        textString="PT1"),
      Text(extent={{-150.0,-150.0},{150.0,-110.0}},
        textString="T=%T")}));
    end FirstOrder;

    block SecondOrder "Second order transfer function block (= 2 poles)"
      import Blocks.Types.Init;
      parameter Real k(unit="1")=1 "Gain";
      parameter Real w(start=1) "Angular frequency";
      parameter Real D(start=1) "Damping";
      parameter Init initType=Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3/4: initial output)" annotation(Evaluate=true,
          Dialog(group="Initialization"));
      parameter Real y_start=0 "Initial or guess value of output (= state)"
        annotation (Dialog(group="Initialization"));
      parameter Real yd_start=0
        "Initial or guess value of derivative of output (= state)"
        annotation (Dialog(group="Initialization"));

      extends Interfaces.SISO(y(start=y_start));
      output Real yd(start=yd_start) "Derivative of y";

    initial equation
      if initType == Init.SteadyState then
        der(y) = 0;
        der(yd) = 0;
      elseif initType == Init.InitialState or initType == Init.InitialOutput then
        y = y_start;
        yd = yd_start;
      end if;
    equation
      der(y) = yd;
      der(yd) = w*(w*(k*u - y) - 2*D*yd);
      annotation (
        Documentation(info="<html>
<p>
This blocks defines the transfer function between the input u and
the output y as <em>second order</em> system:
</p>
<blockquote><pre>
                    k
y = --------------------------------- * u
     ( s / w )^2 + 2*D*( s / w ) + 1
</pre></blockquote>
<p>
If you would like to be able to change easily between different
transfer functions (FirstOrder, SecondOrder, ... ) by changing
parameters, use the general model class <strong>TransferFunction</strong>
instead and model a second order SISO system with parameters<br>
b = {k}, a = {1/w^2, 2*D/w, 1}.
</p>
<blockquote><pre>
Example:

   parameter: k =  0.3,  w = 0.5,  D = 0.4
   results in:
                  0.3
      y = ------------------- * u
          4.0 s^2 + 1.6 s + 1
</pre></blockquote>

</html>"),   Icon(
          coordinateSystem(preserveAspectRatio=true,
                extent={{-100.0,-100.0},{100.0,100.0}}),
              graphics={
          Line(points={{-80.0,78.0},{-80.0,-90.0}},
              color={192,192,192}),
        Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{-80.0,90.0},{-88.0,68.0},{-72.0,68.0},{-80.0,90.0}}),
        Line(points={{-90.0,-80.0},{82.0,-80.0}},
            color={192,192,192}),
        Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{90.0,-80.0},{68.0,-72.0},{68.0,-88.0},{90.0,-80.0}}),
        Line(origin = {-1.939,-1.816},
            points = {{81.939,36.056},{65.362,36.056},{14.39,-26.199},{-29.966,113.485},{-65.374,-61.217},{-78.061,-78.184}},
            color = {0,0,127},
            smooth = Smooth.Bezier),
        Text(textColor={192,192,192},
            extent={{0.0,-70.0},{60.0,-10.0}},
            textString="PT2"),
        Text(extent={{-150.0,-150.0},{150.0,-110.0}},
            textString="w=%w")}));
    end SecondOrder;

    block PI "Proportional-Integral controller"
      import Blocks.Types.Init;
      parameter Real k(unit="1")=1 "Gain";
      parameter SI.Time T(start=1,min=Modelica.Constants.small)
        "Time Constant (T>0 required)";
      parameter Init initType=Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
                                                                                annotation(Evaluate=true,
          Dialog(group="Initialization"));
      parameter Real x_start=0 "Initial or guess value of state"
        annotation (Dialog(group="Initialization"));
      parameter Real y_start=0 "Initial value of output"
        annotation(Dialog(enable=initType == Init.SteadyState or initType == Init.InitialOutput, group=
              "Initialization"));

      extends Interfaces.SISO;
      output Real x(start=x_start) "State of block";

    initial equation
      if initType == Init.SteadyState then
        der(x) = 0;
      elseif initType == Init.InitialState then
        x = x_start;
      elseif initType == Init.InitialOutput then
        y = y_start;
      end if;
    equation
      der(x) = u/T;
      y = k*(x + u);
      annotation (defaultComponentName="PI",
        Documentation(info="<html>
<p>
This blocks defines the transfer function between the input u and
the output y as <em>PI</em> system:
</p>
<blockquote><pre>
              1
y = k * (1 + ---) * u
             T*s
        T*s + 1
  = k * ------- * u
          T*s
</pre></blockquote>
<p>
If you would like to be able to change easily between different
transfer functions (FirstOrder, SecondOrder, ... ) by changing
parameters, use the general model class <strong>TransferFunction</strong>
instead and model a PI SISO system with parameters<br>
b = {k*T, k}, a = {T, 0}.
</p>
<blockquote><pre>
Example:

   parameter: k = 0.3,  T = 0.4

   results in:
               0.4 s + 1
      y = 0.3 ----------- * u
                 0.4 s
</pre></blockquote>

<p>
It might be difficult to initialize the PI component in steady state
due to the integrator part.
This is discussed in the description of package
<a href=\"modelica://Modelica.Blocks.Continuous#info\">Continuous</a>.
</p>

</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,78},{-80,-90}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-80},{82,-80}}, color={192,192,192}),
            Polygon(
              points={{90,-80},{68,-72},{68,-88},{90,-80}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points = {{-80.0,-80.0},{-80.0,-20.0},{60.0,80.0}}, color = {0,0,127}),
            Text(
              extent={{0,6},{60,-56}},
              textColor={192,192,192},
              textString="PI"),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="T=%T")}));
    end PI;

    block PID "PID-controller in additive description form"
      import Blocks.Types.Init;
      extends Interfaces.SISO;

      parameter Real k(unit="1")=1 "Gain";
      parameter SI.Time Ti(min=Modelica.Constants.small, start=0.5)
        "Time Constant of Integrator";
      parameter SI.Time Td(min=0, start=0.1)
        "Time Constant of Derivative block";
      parameter Real Nd(min=Modelica.Constants.small) = 10
        "The higher Nd, the more ideal the derivative block";
      parameter Init initType= Init.InitialState
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
                                         annotation(Evaluate=true,
          Dialog(group="Initialization"));
      parameter Real xi_start=0
        "Initial or guess value for integrator output (= integrator state)"
        annotation (Dialog(group="Initialization"));
      parameter Real xd_start=0
        "Initial or guess value for state of derivative block"
        annotation (Dialog(group="Initialization"));
      parameter Real y_start=0 "Initial value of output"
        annotation(Dialog(enable=initType == Init.InitialOutput, group=
              "Initialization"));
      constant SI.Time unitTime=1 annotation(HideResult=true);

      Blocks.Math.Gain P(k=1) "Proportional part of PID controller"
        annotation (Placement(transformation(extent={{-60,60},{-20,100}})));
      Blocks.Continuous.Integrator I(
        k=unitTime/Ti,
        y_start=xi_start,
        initType=if initType == Init.SteadyState then Init.SteadyState else if
            initType == Init.InitialState then Init.InitialState else Init.NoInit)
        "Integral part of PID controller"
        annotation (Placement(transformation(extent={{-60,-20},{-20,20}})));
      Blocks.Continuous.Derivative D(
        k=Td/unitTime,
        T=max([Td/Nd,100*Modelica.Constants.eps]),
        x_start=xd_start,
        initType=if initType == Init.SteadyState or initType == Init.InitialOutput
             then Init.SteadyState else if initType == Init.InitialState then
            Init.InitialState else Init.NoInit)
        "Derivative part of PID controller"
        annotation (Placement(transformation(extent={{-60,-100},{-20,-60}})));
      Blocks.Math.Gain Gain(k=k) "Gain of PID controller"
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      Blocks.Math.Add3 Add
        annotation (Placement(transformation(extent={{20,-10},{40,10}})));
    initial equation
      if initType==Init.InitialOutput then
         y = y_start;
      end if;

    equation
      connect(u, P.u) annotation (Line(points={{-120,0},{-80,0},{-80,80},{-64,80}}, color={0,0,127}));
      connect(u, I.u)
        annotation (Line(points={{-120,0},{-64,0}}, color={0,0,127}));
      connect(u, D.u) annotation (Line(points={{-120,0},{-80,0},{-80,-80},{-64,-80}},
                     color={0,0,127}));
      connect(P.y, Add.u1) annotation (Line(points={{-18,80},{0,80},{0,8},{18,8}}, color={0,0,127}));
      connect(I.y, Add.u2)
        annotation (Line(points={{-18,0},{18,0}}, color={0,0,127}));
      connect(D.y, Add.u3) annotation (Line(points={{-18,-80},{0,-80},{0,-8},{18,-8}},
                    color={0,0,127}));
      connect(Add.y, Gain.u)
        annotation (Line(points={{41,0},{58,0}}, color={0,0,127}));
      connect(Gain.y, y)
        annotation (Line(points={{81,0},{110,0}}, color={0,0,127}));
      annotation (defaultComponentName="PID",
        Icon(
            coordinateSystem(preserveAspectRatio=true,
                extent={{-100.0,-100.0},{100.0,100.0}}),
                graphics={
            Line(points={{-80.0,78.0},{-80.0,-90.0}},
                color={192,192,192}),
          Polygon(lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid,
              points={{-80.0,90.0},{-88.0,68.0},{-72.0,68.0},{-80.0,90.0}}),
          Line(points={{-90.0,-80.0},{82.0,-80.0}},
              color={192,192,192}),
          Polygon(lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid,
              points={{90.0,-80.0},{68.0,-72.0},{68.0,-88.0},{90.0,-80.0}}),
          Line(points = {{-80,-80},{-80,-20},{60,80}}, color = {0,0,127}),
          Text(textColor={192,192,192},
              extent={{-20.0,-60.0},{80.0,-20.0}},
              textString="PID"),
          Text(extent={{-150.0,-150.0},{150.0,-110.0}},
              textString="Ti=%Ti")}),
        Documentation(info="<html>
<p>
This is the text-book version of a PID-controller.
For a more practically useful PID-controller, use
block LimPID.
</p>

<p>
The PID block can be initialized in different
ways controlled by parameter <strong>initType</strong>. The possible
values of initType are defined in
<a href=\"modelica://Modelica.Blocks.Types.Init\">Modelica.Blocks.Types.Init</a>.
</p>

<p>
Based on the setting of initType, the integrator (I) and derivative (D)
blocks inside the PID controller are initialized according to the following table:
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <tr><td><strong>initType</strong></td>
      <td><strong>I.initType</strong></td>
      <td><strong>D.initType</strong></td></tr>

  <tr><td><strong>NoInit</strong></td>
      <td>NoInit</td>
      <td>NoInit</td></tr>

  <tr><td><strong>SteadyState</strong></td>
      <td>SteadyState</td>
      <td>SteadyState</td></tr>

  <tr><td><strong>InitialState</strong></td>
      <td>InitialState</td>
      <td>InitialState</td></tr>

  <tr><td><strong>InitialOutput</strong><br>
          and initial equation: y = y_start</td>
      <td>NoInit</td>
      <td>SteadyState</td></tr>
</table>

<p>
In many cases, the most useful initial condition is
<strong>SteadyState</strong> because initial transients are then no longer
present. If initType = Init.SteadyState, then in some
cases difficulties might occur. The reason is the
equation of the integrator:
</p>

<blockquote><pre>
<strong>der</strong>(y) = k*u;
</pre></blockquote>

<p>
The steady state equation \"der(x)=0\" leads to the condition that the input u to the
integrator is zero. If the input u is already (directly or indirectly) defined
by another initial condition, then the initialization problem is <strong>singular</strong>
(has none or infinitely many solutions). This situation occurs often
for mechanical systems, where, e.g., u = desiredSpeed - measuredSpeed and
since speed is both a state and a derivative, it is natural to
initialize it with zero. As sketched this is, however, not possible.
The solution is to not initialize u or the variable that is used
to compute u by an algebraic equation.
</p>

</html>"));
    end PID;

    block LimPID
      "P, PI, PD, and PID controller with limited output, anti-windup compensation, setpoint weighting and optional feed-forward"
      import Blocks.Types.Init;
      import Blocks.Types.SimpleController;
      extends Blocks.Interfaces.SVcontrol;
      output Real controlError = u_s - u_m
        "Control error (set point - measurement)";
      parameter .Blocks.Types.SimpleController controllerType=.Blocks.Types.SimpleController.PID
        "Type of controller";
      parameter Real k(min=0, unit="1") = 1 "Gain of controller";
      parameter SI.Time Ti(min=Modelica.Constants.small)=0.5
        "Time constant of Integrator block" annotation (Dialog(enable=
              controllerType == .Blocks.Types.SimpleController.PI or
              controllerType == .Blocks.Types.SimpleController.PID));
      parameter SI.Time Td(min=0)=0.1
        "Time constant of Derivative block" annotation (Dialog(enable=
              controllerType == .Blocks.Types.SimpleController.PD or
              controllerType == .Blocks.Types.SimpleController.PID));
      parameter Real yMax(start=1) "Upper limit of output";
      parameter Real yMin=-yMax "Lower limit of output";
      parameter Real wp(min=0) = 1
        "Set-point weight for Proportional block (0..1)";
      parameter Real wd(min=0) = 0 "Set-point weight for Derivative block (0..1)"
         annotation(Dialog(enable=controllerType == .Blocks.Types.SimpleController.PD
               or controllerType == .Blocks.Types.SimpleController.PID));
      parameter Real Ni(min=100*Modelica.Constants.eps) = 0.9
        "Ni*Ti is time constant of anti-windup compensation"
         annotation(Dialog(enable=controllerType == .Blocks.Types.SimpleController.PI
               or controllerType == .Blocks.Types.SimpleController.PID));
      parameter Real Nd(min=100*Modelica.Constants.eps) = 10
        "The higher Nd, the more ideal the derivative block"
         annotation(Dialog(enable=controllerType == .Blocks.Types.SimpleController.PD
               or controllerType == .Blocks.Types.SimpleController.PID));
      parameter Boolean withFeedForward=false "Use feed-forward input?"
        annotation(Evaluate=true, choices(checkBox=true));
      parameter Real kFF=1 "Gain of feed-forward input"
        annotation(Dialog(enable=withFeedForward));
      parameter Init initType = Init.InitialState
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation(Evaluate=true, Dialog(group="Initialization"));
      parameter Real xi_start=0
        "Initial or guess value for integrator output (= integrator state)"
        annotation (Dialog(group="Initialization",
                    enable=controllerType == .Blocks.Types.SimpleController.PI
               or controllerType == .Blocks.Types.SimpleController.PID));
      parameter Real xd_start=0
        "Initial or guess value for state of derivative block"
        annotation (Dialog(group="Initialization",
                             enable=controllerType == .Blocks.Types.SimpleController.PD
               or controllerType == .Blocks.Types.SimpleController.PID));
      parameter Real y_start=0 "Initial value of output"
        annotation(Dialog(enable=initType == Init.InitialOutput, group=
              "Initialization"));
      parameter Blocks.Types.LimiterHomotopy homotopyType=Blocks.Types.LimiterHomotopy.Linear
        "Simplified model for homotopy-based initialization"
        annotation (Evaluate=true, Dialog(group="Initialization"));
      parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
        annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
      constant SI.Time unitTime=1 annotation (HideResult=true);
      Blocks.Interfaces.RealInput u_ff if withFeedForward
        "Optional connector of feed-forward input signal" annotation (Placement(
            transformation(
            origin={60,-120},
            extent={{20,-20},{-20,20}},
            rotation=270)));
      Blocks.Math.Add addP(k1=wp, k2=-1)
        annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
      Blocks.Math.Add addD(k1=wd, k2=-1) if with_D
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
      Blocks.Math.Gain P(k=1)
        annotation (Placement(transformation(extent={{-50,40},{-30,60}})));
      Blocks.Continuous.Integrator I(
        k=unitTime/Ti,
        y_start=xi_start,
        initType=if initType == Init.SteadyState then Init.SteadyState else if
            initType == Init.InitialState then Init.InitialState else Init.NoInit)
        if with_I
        annotation (Placement(transformation(extent={{-50,-60},{-30,-40}})));
      Blocks.Continuous.Derivative D(
        k=Td/unitTime,
        T=max([Td/Nd,1.e-14]),
        x_start=xd_start,
        initType=if initType == Init.SteadyState or initType == Init.InitialOutput
             then Init.SteadyState else if initType == Init.InitialState then
            Init.InitialState else Init.NoInit) if with_D
        annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
      Blocks.Math.Gain gainPID(k=k)
        annotation (Placement(transformation(extent={{20,-10},{40,10}})));
      Blocks.Math.Add3 addPID
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Blocks.Math.Add3 addI(k2=-1) if with_I
        annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
      Blocks.Math.Add addSat(k1=+1, k2=-1) if with_I annotation (Placement(
            transformation(
            origin={80,-50},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Blocks.Math.Gain gainTrack(k=1/(k*Ni)) if with_I
        annotation (Placement(transformation(extent={{0,-80},{-20,-60}})));
      Blocks.Nonlinear.Limiter limiter(
        uMax=yMax,
        uMin=yMin,
        strict=strict,
        homotopyType=homotopyType)
        annotation (Placement(transformation(extent={{70,-10},{90,10}})));
    protected
      parameter Boolean with_I = controllerType==SimpleController.PI or
                                 controllerType==SimpleController.PID annotation(Evaluate=true, HideResult=true);
      parameter Boolean with_D = controllerType==SimpleController.PD or
                                 controllerType==SimpleController.PID annotation(Evaluate=true, HideResult=true);
    public
      Blocks.Sources.Constant Dzero(k=0) if not with_D
        annotation (Placement(transformation(extent={{-40,20},{-30,30}})));
      Blocks.Sources.Constant Izero(k=0) if not with_I
        annotation (Placement(transformation(extent={{0,-55},{-10,-45}})));
      Blocks.Sources.Constant FFzero(k=0) if not withFeedForward
        annotation (Placement(transformation(extent={{30,-35},{40,-25}})));
      Blocks.Math.Add addFF(k1=1, k2=kFF)
        annotation (Placement(transformation(extent={{48,-6},{60,6}})));
    initial equation
      if initType==Init.InitialOutput then
        gainPID.y = y_start;
      end if;
    equation
      if initType == Init.InitialOutput and (y_start < yMin or y_start > yMax) then
          Modelica.Utilities.Streams.error("LimPID: Start value y_start (=" + String(y_start) +
             ") is outside of the limits of yMin (=" + String(yMin) +") and yMax (=" + String(yMax) + ")");
      end if;

      connect(u_s, addP.u1) annotation (Line(points={{-120,0},{-96,0},{-96,56},{
              -82,56}}, color={0,0,127}));
      connect(u_s, addD.u1) annotation (Line(points={{-120,0},{-96,0},{-96,6},{
              -82,6}}, color={0,0,127}));
      connect(u_s, addI.u1) annotation (Line(points={{-120,0},{-96,0},{-96,-42},{
              -82,-42}}, color={0,0,127}));
      connect(addP.y, P.u) annotation (Line(points={{-59,50},{-52,50}}, color={0,
              0,127}));
      connect(addD.y, D.u)
        annotation (Line(points={{-59,0},{-52,0}}, color={0,0,127}));
      connect(addI.y, I.u) annotation (Line(points={{-59,-50},{-52,-50}}, color={
              0,0,127}));
      connect(P.y, addPID.u1) annotation (Line(points={{-29,50},{-20,50},{-20,8},{-12,
              8}},     color={0,0,127}));
      connect(D.y, addPID.u2)
        annotation (Line(points={{-29,0},{-12,0}},color={0,0,127}));
      connect(I.y, addPID.u3) annotation (Line(points={{-29,-50},{-20,-50},{-20,-8},
              {-12,-8}},    color={0,0,127}));
      connect(limiter.y, addSat.u1) annotation (Line(points={{91,0},{94,0},{94,
              -20},{86,-20},{86,-38}}, color={0,0,127}));
      connect(limiter.y, y)
        annotation (Line(points={{91,0},{110,0}}, color={0,0,127}));
      connect(addSat.y, gainTrack.u) annotation (Line(points={{80,-61},{80,-70},{2,-70}},
                        color={0,0,127}));
      connect(gainTrack.y, addI.u3) annotation (Line(points={{-21,-70},{-88,-70},{-88,
              -58},{-82,-58}},     color={0,0,127}));
      connect(u_m, addP.u2) annotation (Line(points={{0,-120},{0,-92},{-92,-92},{-92,44},{-82,44}}, color={0,0,127}));
      connect(u_m, addD.u2) annotation (Line(points={{0,-120},{0,-92},{-92,-92},{-92,-6},{-82,-6}}, color={0,0,127}));
      connect(u_m, addI.u2) annotation (Line(points={{0,-120},{0,-92},{-92,-92},{-92,-50},{-82,-50}}, color={0,0,127}));
      connect(Dzero.y, addPID.u2) annotation (Line(points={{-29.5,25},{-24,25},{-24,
              0},{-12,0}},    color={0,0,127}));
      connect(Izero.y, addPID.u3) annotation (Line(points={{-10.5,-50},{-20,-50},{-20,
              -8},{-12,-8}},    color={0,0,127}));
      connect(addPID.y, gainPID.u)
        annotation (Line(points={{11,0},{18,0}}, color={0,0,127}));
      connect(addFF.y, limiter.u)
        annotation (Line(points={{60.6,0},{68,0}}, color={0,0,127}));
      connect(gainPID.y, addFF.u1) annotation (Line(points={{41,0},{44,0},{44,3.6},
              {46.8,3.6}},color={0,0,127}));
      connect(FFzero.y, addFF.u2) annotation (Line(points={{40.5,-30},{44,-30},{44,
              -3.6},{46.8,-3.6}},
                            color={0,0,127}));
      connect(addFF.u2, u_ff) annotation (Line(points={{46.8,-3.6},{44,-3.6},{44,
              -92},{60,-92},{60,-120}},
                                   color={0,0,127}));
      connect(addFF.y, addSat.u2) annotation (Line(points={{60.6,0},{64,0},{64,-20},
              {74,-20},{74,-38}}, color={0,0,127}));
      annotation (defaultComponentName="PID",
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,78},{-80,-90}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-80},{82,-80}}, color={192,192,192}),
            Polygon(
              points={{90,-80},{68,-72},{68,-88},{90,-80}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-80},{-80,-20},{30,60},{80,60}}, color={0,0,127}),
            Text(
              extent={{-20,-20},{80,-60}},
              textColor={192,192,192},
              textString="%controllerType"),
            Line(
              visible=strict,
              points={{30,60},{81,60}},
              color={255,0,0})}),
        Diagram(graphics={Text(
                extent={{79,-112},{129,-102}},
                textColor={0,0,255},
              textString=" (feed-forward)")}),
        Documentation(info="<html>
<p>
Via parameter <strong>controllerType</strong> either <strong>P</strong>, <strong>PI</strong>, <strong>PD</strong>,
or <strong>PID</strong> can be selected. If, e.g., PI is selected, all components belonging to the
D-part are removed from the block (via conditional declarations).
The example model
<a href=\"modelica://Modelica.Blocks.Examples.PID_Controller\">Modelica.Blocks.Examples.PID_Controller</a>
demonstrates the usage of this controller.
Several practical aspects of PID controller design are incorporated
according to chapter 3 of the book:
</p>

<dl>
<dt>&Aring;str&ouml;m K.J., and H&auml;gglund T.:</dt>
<dd> <strong>PID Controllers: Theory, Design, and Tuning</strong>.
     Instrument Society of America, 2nd edition, 1995.
</dd>
</dl>

<p>
Besides the additive <strong>proportional, integral</strong> and <strong>derivative</strong>
part of this controller, the following features are present:
</p>
<ul>
<li> The output of this controller is limited. If the controller is
     in its limits, anti-windup compensation is activated to drive
     the integrator state to zero.</li>
<li> The high-frequency gain of the derivative part is limited
     to avoid excessive amplification of measurement noise.</li>
<li> Setpoint weighting is present, which allows to weight
     the setpoint in the proportional and the derivative part
     independently from the measurement. The controller will respond
     to load disturbances and measurement noise independently of this setting
     (parameters wp, wd). However, setpoint changes will depend on this
     setting. For example, it is useful to set the setpoint weight wd
     for the derivative part to zero, if steps may occur in the
     setpoint signal.</li>
<li> Optional feed-forward. It is possible to add a feed-forward signal.
     The feed-forward signal is added before limitation.</li>
</ul>

<p>
The parameters of the controller can be manually adjusted by performing
simulations of the closed loop system (= controller + plant connected
together) and using the following strategy:
</p>

<ol>
<li> Set very large limits, e.g., yMax = Modelica.Constants.inf</li>
<li> Select a <strong>P</strong>-controller and manually enlarge parameter <strong>k</strong>
     (the total gain of the controller) until the closed-loop response
     cannot be improved any more.</li>
<li> Select a <strong>PI</strong>-controller and manually adjust parameters
     <strong>k</strong> and <strong>Ti</strong> (the time constant of the integrator).
     The first value of Ti can be selected, such that it is in the
     order of the time constant of the oscillations occurring with
     the P-controller. If, e.g., vibrations in the order of T=10 ms
     occur in the previous step, start with Ti=0.01 s.</li>
<li> If you want to make the reaction of the control loop faster
     (but probably less robust against disturbances and measurement noise)
     select a <strong>PID</strong>-Controller and manually adjust parameters
     <strong>k</strong>, <strong>Ti</strong>, <strong>Td</strong> (time constant of derivative block).</li>
<li> Set the limits yMax and yMin according to your specification.</li>
<li> Perform simulations such that the output of the PID controller
     goes in its limits. Tune <strong>Ni</strong> (Ni*Ti is the time constant of
     the anti-windup compensation) such that the input to the limiter
     block (= limiter.u) goes quickly enough back to its limits.
     If Ni is decreased, this happens faster. If Ni=infinity, the
     anti-windup compensation is switched off and the controller works bad.</li>
</ol>

<p>
<strong>Initialization</strong>
</p>

<p>
This block can be initialized in different
ways controlled by parameter <strong>initType</strong>. The possible
values of initType are defined in
<a href=\"modelica://Modelica.Blocks.Types.Init\">Modelica.Blocks.Types.Init</a>.
</p>

<p>
Based on the setting of initType, the integrator (I) and derivative (D)
blocks inside the PID controller are initialized according to the following table:
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <tr><td><strong>initType</strong></td>
      <td><strong>I.initType</strong></td>
      <td><strong>D.initType</strong></td></tr>

  <tr><td><strong>NoInit</strong></td>
      <td>NoInit</td>
      <td>NoInit</td></tr>

  <tr><td><strong>SteadyState</strong></td>
      <td>SteadyState</td>
      <td>SteadyState</td></tr>

  <tr><td><strong>InitialState</strong></td>
      <td>InitialState</td>
      <td>InitialState</td></tr>

  <tr><td><strong>InitialOutput</strong><br>
          and initial equation: y = y_start</td>
      <td>NoInit</td>
      <td>SteadyState</td></tr>
</table>

<p>
In many cases, the most useful initial condition is
<strong>SteadyState</strong> because initial transients are then no longer
present. If initType = Init.SteadyState, then in some
cases difficulties might occur. The reason is the
equation of the integrator:
</p>

<blockquote><pre>
<strong>der</strong>(y) = k*u;
</pre></blockquote>

<p>
The steady state equation \"der(x)=0\" leads to the condition that the input u to the
integrator is zero. If the input u is already (directly or indirectly) defined
by another initial condition, then the initialization problem is <strong>singular</strong>
(has none or infinitely many solutions). This situation occurs often
for mechanical systems, where, e.g., u = desiredSpeed - measuredSpeed and
since speed is both a state and a derivative, it is natural to
initialize it with zero. As sketched this is, however, not possible.
The solution is to not initialize u_m or the variable that is used
to compute u_m by an algebraic equation.
</p>

<p>
When initializing in steady-state, homotopy-based initialization can help the convergence of the solver,
by using a simplified model a the beginning of the solution process. Different options are available.
</p>

<ul>
<li><strong>homotopyType=Linear</strong> (default): the limitations are removed from the simplified model,
making it linear. Use this if you know that the controller will not be saturated at steady state.</li>
<li><strong>homotopyType=UpperLimit</strong>: if it is known a priori the controller will be stuck at the upper
limit yMax, this option assumes y = yMax as a simplified model.</li>
<li><strong>homotopyType=LowerLimit</strong>: if it is known a priori the controller will be stuck at the lower
limit yMin, this option assumes y = yMin as a simplified model.</li>
<li><strong>homotopyType=NoHomotopy</strong>: this option does not apply any simplification and keeps the
limiter active throughout the homotopy transformation. Use this if it is unknown whether the controller
is saturated or not at initialization and if the limitations on the output must be enforced throughout
the entire homotopy transformation.</li>
</ul>
</html>"));
    end LimPID;

    block TransferFunction "Linear transfer function"
      import Blocks.Types.Init;
      extends Interfaces.SISO;

      parameter Real b[:]={1}
        "Numerator coefficients of transfer function (e.g., 2*s+3 is specified as {2,3})";
      parameter Real a[:]={1}
        "Denominator coefficients of transfer function (e.g., 5*s+6 is specified as {5,6})";
      parameter Blocks.Types.Init initType=Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation (Evaluate=true, Dialog(group="Initialization"));
      parameter Real x_start[size(a, 1) - 1]=zeros(nx)
        "Initial or guess values of states"
        annotation (Dialog(group="Initialization"));
      parameter Real y_start=0
        "Initial value of output (derivatives of y are zero up to nx-1-th derivative)"
        annotation(Dialog(enable=initType == Init.InitialOutput, group=
              "Initialization"));
      output Real x[size(a, 1) - 1](start=x_start)
        "State of transfer function from controller canonical form";
    protected
      parameter Integer na=size(a, 1) "Size of Denominator of transfer function.";
      parameter Integer nb=size(b, 1) "Size of Numerator of transfer function.";
      parameter Integer nx=size(a, 1) - 1;
      parameter Real bb[:] = vector([zeros(max(0,na-nb),1);b]);
      parameter Real d = bb[1]/a[1];
      parameter Real a_end = if a[end] > 100*Modelica.Constants.eps*sqrt(a*a) then a[end] else 1.0;
      Real x_scaled[size(x,1)] "Scaled vector x";

    initial equation
      if initType == Init.SteadyState then
        der(x_scaled) = zeros(nx);
      elseif initType == Init.InitialState then
        x_scaled = x_start*a_end;
      elseif initType == Init.InitialOutput then
        y = y_start;
        der(x_scaled[2:nx]) = zeros(nx-1);
      end if;
    equation
      assert(size(b,1) <= size(a,1), "Transfer function is not proper");
      if nx == 0 then
         y = d*u;
      else
         der(x_scaled[1])    = (-a[2:na]*x_scaled + a_end*u)/a[1];
         der(x_scaled[2:nx]) = x_scaled[1:nx-1];
         y = ((bb[2:na] - d*a[2:na])*x_scaled)/a_end + d*u;
         x = x_scaled/a_end;
      end if;
      annotation (
        Documentation(info="<html>
<p>
This block defines the transfer function between the input
u and the output y
as (nb = dimension of b, na = dimension of a):
</p>
<blockquote><pre>
        b[1]*s^[nb-1] + b[2]*s^[nb-2] + ... + b[nb]
y(s) = --------------------------------------------- * u(s)
        a[1]*s^[na-1] + a[2]*s^[na-2] + ... + a[na]
</pre></blockquote>
<p>
State variables <strong>x</strong> are defined according to <strong>controller canonical</strong>
form. Internally, vector <strong>x</strong> is scaled to improve the numerics (the states in versions before version 3.0 of the Modelica Standard Library have been not scaled). This scaling is
not visible from the outside of this block because the non-scaled vector <strong>x</strong>
is provided as output signal and the start value is with respect to the non-scaled
vector <strong>x</strong>.
Initial values of the states <strong>x</strong> can be set via parameter <strong>x_start</strong>.
</p>

<p>
Example:
</p>
<blockquote><pre>
TransferFunction g(b = {2,4}, a = {1,3});
</pre></blockquote>
<p>
results in the following transfer function:
</p>
<blockquote><pre>
     2*s + 4
y = --------- * u
      s + 3
</pre></blockquote>
</html>"),
        Icon(
            coordinateSystem(preserveAspectRatio=true,
              extent={{-100.0,-100.0},{100.0,100.0}}),
              graphics={
            Line(points={{-80.0,0.0},{80.0,0.0}},
              color={0,0,127}),
          Text(textColor={0,0,127},
            extent={{-90.0,10.0},{90.0,90.0}},
            textString="b(s)"),
          Text(textColor={0,0,127},
            extent={{-90.0,-90.0},{90.0,-10.0}},
            textString="a(s)")}));
    end TransferFunction;

    block StateSpace "Linear state space system"
      import Blocks.Types.Init;
      parameter Real A[:, size(A, 1)]=[1, 0; 0, 1]
        "Matrix A of state space model (e.g., A=[1, 0; 0, 1])";
      parameter Real B[size(A, 1), :]=[1; 1]
        "Matrix B of state space model (e.g., B=[1; 1])";
      parameter Real C[:, size(A, 1)]=[1, 1]
        "Matrix C of state space model (e.g., C=[1, 1])";
      parameter Real D[size(C, 1), size(B, 2)]=zeros(size(C, 1), size(B, 2))
        "Matrix D of state space model";
      parameter Blocks.Types.Init initType=Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation (Evaluate=true, Dialog(group="Initialization"));
      parameter Real x_start[nx]=zeros(nx) "Initial or guess values of states"
        annotation (Dialog(group="Initialization"));
      parameter Real y_start[ny]=zeros(ny)
        "Initial values of outputs (remaining states are in steady state if possible)"
        annotation(Dialog(enable=initType == Init.InitialOutput, group=
              "Initialization"));

      extends Interfaces.MIMO(final nin=size(B, 2), final nout=size(C, 1));
      output Real x[size(A, 1)](start=x_start) "State vector";

    protected
      parameter Integer nx = size(A, 1) "Number of states";
      parameter Integer ny = size(C, 1) "Number of outputs";
    initial equation
      if initType == Init.SteadyState then
        der(x) = zeros(nx);
      elseif initType == Init.InitialState then
        x = x_start;
      elseif initType == Init.InitialOutput then
        x = Modelica.Math.Matrices.equalityLeastSquares(A, -B*u, C, y_start - D*u);
      end if;
    equation
      der(x) = A*x + B*u;
      y = C*x + D*u;
      annotation (
        Documentation(info="<html>
<p>
The State Space block defines the relation
between the input u and the output
y in state space form:
</p>
<blockquote><pre>
der(x) = A * x + B * u
    y  = C * x + D * u
</pre></blockquote>
<p>
The input is a vector of length nu, the output is a vector
of length ny and nx is the number of states. Accordingly
</p>
<blockquote><pre>
A has the dimension: A(nx,nx),
B has the dimension: B(nx,nu),
C has the dimension: C(ny,nx),
D has the dimension: D(ny,nu)
</pre></blockquote>
<p>
Example:
</p>
<blockquote><pre>
parameter: A = [0.12, 2;3, 1.5]
parameter: B = [2, 7;3, 1]
parameter: C = [0.1, 2]
parameter: D = zeros(ny,nu)

results in the following equations:
  [der(x[1])]   [0.12  2.00] [x[1]]   [2.0  7.0] [u[1]]
  [         ] = [          ]*[    ] + [        ]*[    ]
  [der(x[2])]   [3.00  1.50] [x[2]]   [0.1  2.0] [u[2]]
                             [x[1]]            [u[1]]
       y[1]   = [0.1  2.0] * [    ] + [0  0] * [    ]
                             [x[2]]            [u[2]]
</pre></blockquote>
</html>"),   Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}),
          graphics={
        Text(extent={{-90,10},{-10,90}},
          textString="A",
          textColor={0,0,127}),
        Text(extent={{10,10},{90,90}},
          textString="B",
          textColor={0,0,127}),
        Text(extent={{-90,-10},{-10,-90}},
          textString="C",
          textColor={0,0,127}),
        Text(extent={{10,-10},{90,-90}},
          textString="D",
          textColor={0,0,127}),
        Line(points={{0,-90},{0,90}},
          color={192,192,192}),
        Line(points={{-90,0},{90,0}},
          color={192,192,192})}));
    end StateSpace;

    block Der "Derivative of input (= analytic differentiations)"
        extends Interfaces.SISO;

    equation
      y = der(u);
        annotation (defaultComponentName="der1",
     Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
            graphics={Text(
              extent={{-96,28},{94,-24}},
              textString="der()",
              textColor={0,0,127})}),
            Documentation(info="<html>
<p>
Defines that the output y is the <em>derivative</em>
of the input u. Note, that Modelica.Blocks.Continuous.Derivative
computes the derivative in an approximate sense, where as this block computes
the derivative exactly. This requires that the input u is differentiated
by the Modelica translator, if this derivative is not yet present in
the model.
</p>
</html>"));
    end Der;

    block LowpassButterworth
      "Output the input signal filtered with a low pass Butterworth filter of any order"

      import Blocks.Types.Init;
      import Modelica.Constants.pi;

      extends Blocks.Interfaces.SISO;

      parameter Integer n(min=1) = 2 "Order of filter";
      parameter SI.Frequency f(start=1) "Cut-off frequency";
      parameter Blocks.Types.Init initType=Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation (Evaluate=true, Dialog(group="Initialization"));
      parameter Real x1_start[m]=zeros(m)
        "Initial or guess values of states 1 (der(x1)=x2)"
        annotation (Dialog(group="Initialization"));
      parameter Real x2_start[m]=zeros(m) "Initial or guess values of states 2"
        annotation (Dialog(group="Initialization"));
      parameter Real xr_start=0.0
        "Initial or guess value of real pole for uneven order otherwise dummy"
        annotation (Dialog(group="Initialization"));
      parameter Real y_start=0.0
        "Initial value of output (states are initialized in steady state if possible)"
         annotation(Dialog(enable=initType == Init.InitialOutput, group=
              "Initialization"));

      output Real x1[m](start=x1_start)
        "States 1 of second order filters (der(x1) = x2)";
      output Real x2[m](start=x2_start) "States 2 of second order filters";
      output Real xr(start=xr_start)
        "State of real pole for uneven order otherwise dummy";
    protected
      parameter Integer m=integer(n/2);
      parameter Boolean evenOrder = 2*m == n;
      parameter Real w=2*pi*f;
      Real z[m + 1];
      Real polereal[m];
      Real poleimag[m];
      Real realpol;
      Real k2[m];
      Real D[m];
      Real w0[m];
      Real k1;
      Real T;
    initial equation
      if initType == Init.SteadyState then
        der(x1) = zeros(m);
        der(x2) = zeros(m);
        if not evenOrder then
          der(xr) = 0.0;
        end if;
      elseif initType == Init.InitialState then
        x1 = x1_start;
        x2 = x2_start;
        if not evenOrder then
          xr = xr_start;
        end if;
      elseif initType == Init.InitialOutput then
        y = y_start;
        der(x1) = zeros(m);
        if evenOrder then
          if m > 1 then
            der(x2[1:m-1]) = zeros(m-1);
          end if;
        else
          der(x1) = zeros(m);
        end if;
      end if;
    equation
      k2 = ones(m);
      k1 = 1;
      z[1] = u;

      // calculate filter parameters
      for i in 1:m loop
        // poles of prototype lowpass
        polereal[i] = Modelica.Math.cos(pi/2 + pi/n*(i - 0.5));
        poleimag[i] = Modelica.Math.sin(pi/2 + pi/n*(i - 0.5));
        // scaling and calculation of second order filter coefficients
        w0[i] = (polereal[i]^2 + poleimag[i]^2)*w;
        D[i] = -polereal[i]/w0[i]*w;
      end for;
      realpol = 1*w;
      T = 1/realpol;

      // calculate second order filters
      for i in 1:m loop
        der(x1[i]) = x2[i];
        der(x2[i]) = k2[i]*w0[i]^2*z[i] - 2*D[i]*w0[i]*x2[i] - w0[i]^2*x1[i];
        z[i + 1] = x1[i];
      end for;

      // calculate first order filter if necessary
      if evenOrder then
        // even order
        xr = 0;
        y = z[m + 1];
      else
        // uneven order
        der(xr) = (k1*z[m + 1] - xr)/T;
        y = xr;
      end if;
      annotation (
        Icon(
            coordinateSystem(preserveAspectRatio=true,
                extent={{-100.0,-100.0},{100.0,100.0}}),
                graphics={
            Line(points={{-80.0,78.0},{-80.0,-90.0}},
                color={192,192,192}),
            Polygon(lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid,
                points={{-79.5584,91.817},{-87.5584,69.817},{-71.5584,69.817},{-79.5584,91.817}}),
            Line(origin = {-1.939,-1.816},
                points = {{81.939,36.056},{65.362,36.056},{14.39,-26.199},{-29.966,113.485},{-65.374,-61.217},{-78.061,-78.184}},
                color = {0,0,127},
                smooth = Smooth.Bezier),
            Line(points={{-90.9779,-80.7697},{81.0221,-80.7697}},
                color={192,192,192}),
            Polygon(lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid,
                points={{91.3375,-79.8233},{69.3375,-71.8233},{69.3375,-87.8233},{91.3375,-79.8233}}),
            Text(textColor={192,192,192},
                extent={{-45.1735,-68.0},{92.0,-11.47}},
                textString="LowpassButterworthFilter"),
            Text(extent={{8.0,-146.0},{8.0,-106.0}},
                textString="f=%f"),
            Text(textColor={192,192,192},
                extent={{-2.0,48.0},{94.0,94.0}},
                textString="%n")}),
        Documentation(info="<html>
<p>
This block defines the transfer function between the input u
and the output y as an n-th order low pass filter with <em>Butterworth</em>
characteristics and cut-off frequency f. It is implemented as
a series of second order filters and a first order filter.
Butterworth filters have the feature that the amplitude at the
cut-off frequency f is 1/sqrt(2) (= 3 dB), i.e., they are
always \"normalized\". Step responses of the Butterworth filter of
different orders are shown in the next figure:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/Butterworth.png\"
     alt=\"Butterworth.png\">
</p>

<p>
If transients at the simulation start shall be avoided, the filter
should be initialized in steady state (e.g., using option
initType=Modelica.Blocks.Types.Init.SteadyState).
</p>

</html>"));
    end LowpassButterworth;

    block CriticalDamping
      "Output the input signal filtered with an n-th order filter with critical damping"

      import Blocks.Types.Init;
      extends Blocks.Interfaces.SISO;

      parameter Integer n=2 "Order of filter";
      parameter SI.Frequency f(start=1) "Cut-off frequency";
      parameter Boolean normalized = true
        "= true, if amplitude at f_cut is 3 dB, otherwise unmodified filter";
      parameter Blocks.Types.Init initType=Blocks.Types.Init.NoInit
        "Type of initialization (1: no init, 2: steady state, 3: initial state, 4: initial output)"
        annotation (Evaluate=true, Dialog(group="Initialization"));
      parameter Real x_start[n]=zeros(n) "Initial or guess values of states"
        annotation (Dialog(group="Initialization"));
      parameter Real y_start=0.0
        "Initial value of output (remaining states are in steady state)"
        annotation(Dialog(enable=initType == Init.InitialOutput, group=
              "Initialization"));

      output Real x[n](start=x_start) "Filter states";
    protected
      parameter Real alpha=if normalized then sqrt(2^(1/n) - 1) else 1.0
        "Frequency correction factor for normalized filter";
      parameter Real w=2*Modelica.Constants.pi*f/alpha;
    initial equation
      if initType == Init.SteadyState then
        der(x) = zeros(n);
      elseif initType == Init.InitialState then
        x = x_start;
      elseif initType == Init.InitialOutput then
        y = y_start;
        der(x[1:n-1]) = zeros(n-1);
      end if;
    equation
      der(x[1]) = (u - x[1])*w;
      for i in 2:n loop
        der(x[i]) = (x[i - 1] - x[i])*w;
      end for;
      y = x[n];
      annotation (
        Icon(
            coordinateSystem(preserveAspectRatio=true,
              extent={{-100.0,-100.0},{100.0,100.0}}),
              graphics={
            Line(points={{-80.6897,77.6256},{-80.6897,-90.3744}},
              color={192,192,192}),
            Polygon(lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid,
              points={{-79.7044,90.6305},{-87.7044,68.6305},{-71.7044,68.6305},{-79.7044,90.6305}}),
            Line(points={{-90.0,-80.0},{82.0,-80.0}},
              color={192,192,192}),
            Polygon(lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid,
              points={{90.0,-80.0},{68.0,-72.0},{68.0,-88.0},{90.0,-80.0}}),
            Text(textColor={192,192,192},
              extent={{0.0,-60.0},{60.0,0.0}},
              textString="PTn"),
            Line(origin = {-17.976,-6.521},
              points = {{96.962,55.158},{16.42,50.489},{-18.988,18.583},{-32.024,-53.479},{-62.024,-73.479}},
              color = {0,0,127},
              smooth = Smooth.Bezier),
            Text(textColor={192,192,192},
              extent={{-70.0,48.0},{26.0,94.0}},
              textString="%n"),
            Text(extent={{8.0,-146.0},{8.0,-106.0}},
              textString="f=%f")}),
        Documentation(info="<html>
<p>This block defines the transfer function between the
input u and the output y
as an n-th order filter with <em>critical damping</em>
characteristics and cut-off frequency f. It is
implemented as a series of first order filters.
This filter type is especially useful to filter the input of an
inverse model, since the filter does not introduce any transients.
</p>

<p>
If parameter <strong>normalized</strong> = <strong>true</strong> (default), the filter
is normalized such that the amplitude of the filter transfer function
at the cut-off frequency f is 1/sqrt(2) (= 3 dB). Otherwise, the filter
is not normalized, i.e., it is unmodified. A normalized filter is usually
much better for applications, since filters of different orders are
\"comparable\", whereas non-normalized filters usually require to adapt the
cut-off frequency, when the order of the filter is changed.
Figures of the filter step responses are shown below.
Note, in versions before version 3.0 of the Modelica Standard library,
the CriticalDamping filter was provided only in non-normalized form.
</p>

<p>If transients at the simulation start shall be avoided, the filter
should be initialized in steady state (e.g., using option
initType=Modelica.Blocks.Types.Init.SteadyState).
</p>

<p>
The critical damping filter is defined as
</p>

<blockquote><pre>
&alpha; = <strong>if</strong> normalized <strong>then</strong> <strong>sqrt</strong>(2^(1/n) - 1) <strong>else</strong> 1 // frequency correction factor
&omega; = 2*&pi;*f/&alpha;
          1
y = ------------- * u
     (s/w + 1)^n

</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/CriticalDampingNormalized.png\"
     alt=\"CriticalDampingNormalized.png\">
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/CriticalDampingNonNormalized.png\"
     alt=\"CriticalDampingNonNormalized.png\">
</p>

</html>"));
    end CriticalDamping;

    block Filter
      "Continuous low pass, high pass, band pass or band stop IIR-filter of type CriticalDamping, Bessel, Butterworth or ChebyshevI"
      import Blocks.Continuous.Internal;

      extends Blocks.Interfaces.SISO;

      parameter Blocks.Types.AnalogFilter analogFilter=Blocks.Types.AnalogFilter.CriticalDamping
        "Analog filter characteristics (CriticalDamping/Bessel/Butterworth/ChebyshevI)";
      parameter Blocks.Types.FilterType filterType=Blocks.Types.FilterType.LowPass
        "Type of filter (LowPass/HighPass/BandPass/BandStop)";
      parameter Integer order(min=1) = 2 "Order of filter";
      parameter SI.Frequency f_cut "Cut-off frequency";
      parameter Real gain=1.0
        "Gain (= amplitude of frequency response at zero frequency)";
      parameter Real A_ripple(unit="dB") = 0.5
        "Pass band ripple for Chebyshev filter (otherwise not used); > 0 required"
        annotation(Dialog(enable=analogFilter == Blocks.Types.AnalogFilter.ChebyshevI));
      parameter SI.Frequency f_min=0
        "Band of band pass/stop filter is f_min (A=-3db*gain) .. f_cut (A=-3db*gain)"
        annotation(Dialog(enable=filterType == Blocks.Types.FilterType.BandPass
               or filterType == Blocks.Types.FilterType.BandStop));
      parameter Boolean normalized=true
        "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";
      parameter Blocks.Types.Init init=Blocks.Types.Init.SteadyState
        "Type of initialization (no init/steady state/initial state/initial output)"
        annotation (Evaluate=true, Dialog(tab="Advanced"));
      final parameter Integer nx = if filterType ==Blocks.Types.FilterType.LowPass           or
                                      filterType ==Blocks.Types.FilterType.HighPass           then
                                      order else 2*order;
      parameter Real x_start[nx] = zeros(nx) "Initial or guess values of states"
        annotation(Dialog(tab="Advanced"));
      parameter Real y_start = 0 "Initial value of output"
        annotation(Dialog(tab="Advanced"));
      parameter Real u_nominal = 1.0
        "Nominal value of input (used for scaling the states)"
      annotation(Dialog(tab="Advanced"));
      Blocks.Interfaces.RealOutput x[nx] "Filter states";

    protected
      parameter Integer ncr = if analogFilter ==Blocks.Types.AnalogFilter.CriticalDamping           then
                                 order else mod(order,2);
      parameter Integer nc0 = if analogFilter ==Blocks.Types.AnalogFilter.CriticalDamping           then
                                 0 else integer(order/2);
      parameter Integer na = if filterType ==Blocks.Types.FilterType.BandPass           or
                                filterType ==Blocks.Types.FilterType.BandStop           then order else
                             if analogFilter ==Blocks.Types.AnalogFilter.CriticalDamping           then
                                0 else integer(order/2);
      parameter Integer nr = if filterType ==Blocks.Types.FilterType.BandPass           or
                                filterType ==Blocks.Types.FilterType.BandStop           then 0 else
                             if analogFilter ==Blocks.Types.AnalogFilter.CriticalDamping           then
                                order else mod(order,2);

      // Coefficients of prototype base filter (low pass filter with w_cut = 1 rad/s)
      parameter Real cr[ncr](each fixed=false);
      parameter Real c0[nc0](each fixed=false);
      parameter Real c1[nc0](each fixed=false);

      // Coefficients for differential equations.
      parameter Real r[nr](each fixed=false);
      parameter Real a[na](each fixed=false);
      parameter Real b[na](each fixed=false);
      parameter Real ku[na](each fixed=false);
      parameter Real k1[if filterType == Blocks.Types.FilterType.LowPass then 0
         else na](each fixed=false);
      parameter Real k2[if filterType == Blocks.Types.FilterType.LowPass then 0
         else na](each fixed=false);

      // Auxiliary variables
      Real uu[na+nr+1];

    initial equation
      if analogFilter == Blocks.Types.AnalogFilter.CriticalDamping then
          cr = Internal.Filter.base.CriticalDamping(order, normalized);
      elseif analogFilter == Blocks.Types.AnalogFilter.Bessel then
          (cr,c0,c1) = Internal.Filter.base.Bessel(order, normalized);
      elseif analogFilter == Blocks.Types.AnalogFilter.Butterworth then
          (cr,c0,c1) = Internal.Filter.base.Butterworth(order, normalized);
      elseif analogFilter == Blocks.Types.AnalogFilter.ChebyshevI then
          (cr,c0,c1) = Internal.Filter.base.ChebyshevI(order, A_ripple, normalized);
       end if;

      if filterType == Blocks.Types.FilterType.LowPass then
          (r,a,b,ku) = Internal.Filter.roots.lowPass(cr,c0,c1,f_cut);
      elseif filterType == Blocks.Types.FilterType.HighPass then
          (r,a,b,ku,k1,k2) = Internal.Filter.roots.highPass(cr,c0,c1,f_cut);
      elseif filterType == Blocks.Types.FilterType.BandPass then
          (a,b,ku,k1,k2) = Internal.Filter.roots.bandPass(cr,c0,c1,f_min,f_cut);
      elseif filterType == Blocks.Types.FilterType.BandStop then
          (a,b,ku,k1,k2) = Internal.Filter.roots.bandStop(cr,c0,c1,f_min,f_cut);
       end if;

      if init == Blocks.Types.Init.InitialState then
          x = x_start;
      elseif init == Blocks.Types.Init.SteadyState then
          der(x) = zeros(nx);
      elseif init == Blocks.Types.Init.InitialOutput then
          y = y_start;
          if nx > 1 then
             der(x[1:nx-1]) = zeros(nx-1);
          end if;
       end if;

    equation
       assert(u_nominal > 0, "u_nominal > 0 required");
      assert(filterType == Blocks.Types.FilterType.LowPass or filterType ==
        Blocks.Types.FilterType.HighPass or f_min > 0,
        "f_min > 0 required for band pass and band stop filter");
       assert(A_ripple > 0, "A_ripple > 0 required");
       assert(f_cut > 0, "f_cut > 0 required");

       /* All filters have the same basic differential equations:
        Real poles:
           der(x) = r*x - r*u
        Complex conjugate poles:
           der(x1) = a*x1 - b*x2 + ku*u;
           der(x2) = b*x1 + a*x2;
   */
       uu[1] = u/u_nominal;
       for i in 1:nr loop
          der(x[i]) = r[i]*(x[i] - uu[i]);
       end for;
       for i in 1:na loop
          der(x[nr+2*i-1]) = a[i]*x[nr+2*i-1] - b[i]*x[nr+2*i] + ku[i]*uu[nr+i];
          der(x[nr+2*i])   = b[i]*x[nr+2*i-1] + a[i]*x[nr+2*i];
       end for;

       // The output equation is different for the different filter types
      if filterType == Blocks.Types.FilterType.LowPass then
          /* Low pass filter
           Real poles             :  y = x
           Complex conjugate poles:  y = x2
      */
          for i in 1:nr loop
             uu[i+1] = x[i];
          end for;
          for i in 1:na loop
             uu[nr+i+1] = x[nr+2*i];
          end for;

      elseif filterType == Blocks.Types.FilterType.HighPass then
          /* High pass filter
           Real poles             :  y = -x + u;
           Complex conjugate poles:  y = k1*x1 + k2*x2 + u;
      */
          for i in 1:nr loop
             uu[i+1] = -x[i] + uu[i];
          end for;
          for i in 1:na loop
             uu[nr+i+1] = k1[i]*x[nr+2*i-1] + k2[i]*x[nr+2*i] + uu[nr+i];
          end for;

      elseif filterType == Blocks.Types.FilterType.BandPass then
          /* Band pass filter
           Complex conjugate poles:  y = k1*x1 + k2*x2;
      */
          for i in 1:na loop
             uu[nr+i+1] = k1[i]*x[nr+2*i-1] + k2[i]*x[nr+2*i];
          end for;

      elseif filterType == Blocks.Types.FilterType.BandStop then
          /* Band pass filter
           Complex conjugate poles:  y = k1*x1 + k2*x2 + u;
      */
          for i in 1:na loop
             uu[nr+i+1] = k1[i]*x[nr+2*i-1] + k2[i]*x[nr+2*i] + uu[nr+i];
          end for;

       else
          assert(false, "filterType (= " + String(filterType) + ") is unknown");
          uu = zeros(na+nr+1);
       end if;

       y = (gain*u_nominal)*uu[nr+na+1];

      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Line(points={{-80.0,80.0},{-80.0,-88.0}},
            color={192,192,192}),
          Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{-80.0,92.0},{-88.0,70.0},{-72.0,70.0},{-80.0,92.0}}),
          Line(points={{-90.0,-78.0},{82.0,-78.0}},
            color={192,192,192}),
          Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{90.0,-78.0},{68.0,-70.0},{68.0,-86.0},{90.0,-78.0}}),
          Text(textColor={192,192,192},
            extent={{-66.0,52.0},{88.0,90.0}},
            textString="%order"),
          Text(
            extent={{-138.0,-140.0},{162.0,-110.0}},
            textString="f_cut=%f_cut"),
          Rectangle(lineColor={160,160,164},
            fillColor={255,255,255},
            fillPattern=FillPattern.Backward,
            extent={{-80.0,-78.0},{22.0,10.0}}),
          Line(origin = {3.333,-6.667}, points = {{-83.333,34.667},{24.667,34.667},{42.667,-71.333}}, color = {0,0,127}, smooth = Smooth.Bezier)}),
        Documentation(info="<html>

<p>
This blocks models various types of filters:
</p>

<blockquote>
<strong>low pass, high pass, band pass, and band stop filters</strong>
</blockquote>

<p>
using various filter characteristics:
</p>

<blockquote>
<strong>CriticalDamping, Bessel, Butterworth, Chebyshev Type I filters</strong>
</blockquote>

<p>
By default, a filter block is initialized in <strong>steady-state</strong>, in order to
avoid unwanted oscillations at the beginning. In special cases, it might be
useful to select one of the other initialization options under tab
\"Advanced\".
</p>

<p>
Typical frequency responses for the 4 supported low pass filter types
are shown in the next figure:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/LowPassOrder4Filters.png\"
     alt=\"LowPassOrder4Filters.png\">
</blockquote>

<p>
The step responses of the same low pass filters are shown in the next figure,
starting from a steady state initial filter with initial input = 0.2:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/LowPassOrder4FiltersStepResponse.png\"
     alt=\"LowPassOrder4FiltersStepResponse.png\">
</blockquote>

<p>
Obviously, the frequency responses give a somewhat wrong impression
of the filter characteristics: Although Butterworth and Chebyshev
filters have a significantly steeper magnitude as the
CriticalDamping and Bessel filters, the step responses of
the latter ones are much better. This means for example, that
a CriticalDamping or a Bessel filter should be selected,
if a filter is mainly used to make a non-linear inverse model
realizable.
</p>

<p>
Typical frequency responses for the 4 supported high pass filter types
are shown in the next figure:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/HighPassOrder4Filters.png\"
     alt=\"HighPassOrder4Filters.png\">
</blockquote>

<p>
The corresponding step responses of these high pass filters are
shown in the next figure:
</p>
<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/HighPassOrder4FiltersStepResponse.png\"
     alt=\"HighPassOrder4FiltersStepResponse.png\">
</blockquote>

<p>
All filters are available in <strong>normalized</strong> (default) and non-normalized form.
In the normalized form, the amplitude of the filter transfer function
at the cut-off frequency f_cut is -3 dB (= 10^(-3/20) = 0.70794..).
Note, when comparing the filters of this function with other software systems,
the setting of \"normalized\" has to be selected appropriately. For example, the signal processing
toolbox of MATLAB provides the filters in non-normalized form and
therefore a comparison makes only sense, if normalized = <strong>false</strong>
is set. A normalized filter is usually better suited for applications,
since filters of different orders are \"comparable\",
whereas non-normalized filters usually require to adapt the
cut-off frequency, when the order of the filter is changed.
See a comparison of \"normalized\" and \"non-normalized\" filters at hand of
CriticalDamping filters of order 1,2,3:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/CriticalDampingNormalized.png\"
     alt=\"CriticalDampingNormalized.png\">
</blockquote>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Continuous/CriticalDampingNonNormalized.png\"
     alt=\"CriticalDampingNonNormalized.png\">
</blockquote>

<h4>Implementation</h4>

<p>
The filters are implemented in the following, reliable way:
</p>

<ol>
<li> A prototype low pass filter with a cut-off angular frequency of 1 rad/s is constructed
     from the desired analogFilter and the desired normalization.</li>

<li> This prototype low pass filter is transformed to the desired filterType and the
     desired cut-off frequency f_cut using a transformation on the Laplace variable \"s\".</li>

<li> The resulting first and second order transfer functions are implemented in
     state space form, using the \"eigen value\" representation of a transfer function:
     <blockquote><pre>
// second order block with eigen values: a +/- jb
<strong>der</strong>(x1) = a*x1 - b*x2 + (a^2 + b^2)/b*u;
<strong>der</strong>(x2) = b*x1 + a*x2;
     y  = x2;
     </pre></blockquote>
     The dc-gain from the input to the output of this block is one and the selected
     states are in the order of the input (if \"u\" is in the order of \"one\", then the
     states are also in the order of \"one\"). In the \"Advanced\" tab, a \"nominal\" value for
     the input \"u\" can be given. If appropriately selected, the states are in the order of \"one\" and
     then step-size control is always appropriate.</li>
</ol>

<h4>References</h4>

<dl>
<dt>Tietze U., and Schenk C. (2002):</dt>
<dd> <strong>Halbleiter-Schaltungstechnik</strong>.
     Springer Verlag, 12. Auflage, pp. 815-852.</dd>
</dl>

</html>",   revisions="<html>
<dl>
  <dt><strong>Main Author:</strong></dt>
  <dd><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>,
      DLR Oberpfaffenhofen.</dd>
</dl>

<h4>Acknowledgement</h4>

<p>
The development of this block was partially funded by BMBF within the
     <a href=\"http://www.eurosyslib.com/\">ITEA2 EUROSYSLIB</a>
      project.
</p>

</html>"));
    end Filter;

    package Internal
      "Internal utility functions and blocks that should not be directly utilized by the user"
        extends Modelica.Icons.InternalPackage;
      package Filter
        "Internal utility functions for filters that should not be directly used"
          extends Modelica.Icons.InternalPackage;
        package base
          "Prototype low pass filters with cut-off frequency of 1 rad/s (other filters are derived by transformation from these base filters)"
            extends Modelica.Icons.InternalPackage;
        function CriticalDamping
            "Return base filter coefficients of CriticalDamping filter (= low pass filter with w_cut = 1 rad/s)"
          extends Modelica.Icons.Function;

          input Integer order(min=1) "Order of filter";
          input Boolean normalized=true
              "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";

          output Real cr[order] "Coefficients of real poles";
          protected
          Real alpha=1.0 "Frequency correction factor";
          Real alpha2 "= alpha*alpha";
          Real den1[order]
              "[p] coefficients of denominator first order polynomials (a*p + 1)";
          Real den2[0,2]
              "[p^2, p] coefficients of denominator second order polynomials (b*p^2 + a*p + 1)";
          Real c0[0] "Coefficients of s^0 term if conjugate complex pole";
          Real c1[0] "Coefficients of s^1 term if conjugate complex pole";
        algorithm
          if normalized then
             // alpha := sqrt(2^(1/order) - 1);
             alpha := sqrt(10^(3/10/order)-1);
          else
             alpha := 1.0;
          end if;

          for i in 1:order loop
             den1[i] := alpha;
          end for;

          // Determine polynomials with highest power of s equal to one
            (cr,c0,c1) :=
              Blocks.Continuous.Internal.Filter.Utilities.toHighestPowerOne(
              den1, den2);
        end CriticalDamping;

        function Bessel
            "Return base filter coefficients of Bessel filter (= low pass filter with w_cut = 1 rad/s)"
          extends Modelica.Icons.Function;

          input Integer order(min=1) "Order of filter";
          input Boolean normalized=true
              "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";

          output Real cr[mod(order, 2)] "Coefficient of real pole";
          output Real c0[integer(order/2)]
              "Coefficients of s^0 term if conjugate complex pole";
          output Real c1[integer(order/2)]
              "Coefficients of s^1 term if conjugate complex pole";
          protected
          Real alpha=1.0 "Frequency correction factor";
          Real alpha2 "= alpha*alpha";
          Real den1[size(cr,1)]
              "[p] coefficients of denominator first order polynomials (a*p + 1)";
          Real den2[size(c0, 1),2]
              "[p^2, p] coefficients of denominator second order polynomials (b*p^2 + a*p + 1)";
        algorithm
            (den1,den2,alpha) :=
              Blocks.Continuous.Internal.Filter.Utilities.BesselBaseCoefficients(
              order);
          if not normalized then
             alpha2 := alpha*alpha;
             for i in 1:size(c0, 1) loop
               den2[i, 1] := den2[i, 1]*alpha2;
               den2[i, 2] := den2[i, 2]*alpha;
             end for;
             if size(cr,1) == 1 then
               den1[1] := den1[1]*alpha;
             end if;
             end if;

          // Determine polynomials with highest power of s equal to one
            (cr,c0,c1) :=
              Blocks.Continuous.Internal.Filter.Utilities.toHighestPowerOne(
              den1, den2);
        end Bessel;

        function Butterworth
            "Return base filter coefficients of Butterworth filter (= low pass filter with w_cut = 1 rad/s)"
          import Modelica.Constants.pi;
          extends Modelica.Icons.Function;

          input Integer order(min=1) "Order of filter";
          input Boolean normalized=true
              "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";

          output Real cr[mod(order, 2)] "Coefficient of real pole";
          output Real c0[integer(order/2)]
              "Coefficients of s^0 term if conjugate complex pole";
          output Real c1[integer(order/2)]
              "Coefficients of s^1 term if conjugate complex pole";
          protected
          Real alpha=1.0 "Frequency correction factor";
          Real alpha2 "= alpha*alpha";
          Real den1[size(cr,1)]
              "[p] coefficients of denominator first order polynomials (a*p + 1)";
          Real den2[size(c0, 1),2]
              "[p^2, p] coefficients of denominator second order polynomials (b*p^2 + a*p + 1)";
        algorithm
          for i in 1:size(c0, 1) loop
            den2[i, 1] := 1.0;
            den2[i, 2] := -2*Modelica.Math.cos(pi*(0.5 + (i - 0.5)/order));
          end for;
          if size(cr,1) == 1 then
            den1[1] := 1.0;
          end if;

          /* Transformation of filter transfer function with "new(p) = alpha*p"
     in order that the filter transfer function has an amplitude of
     -3 db at the cutoff frequency
  */
          /*
    if normalized then
      alpha := Internal.normalizationFactor(den1, den2);
      alpha2 := alpha*alpha;
      for i in 1:size(c0, 1) loop
        den2[i, 1] := den2[i, 1]*alpha2;
        den2[i, 2] := den2[i, 2]*alpha;
      end for;
      if size(cr,1) == 1 then
        den1[1] := den1[1]*alpha;
      end if;
    end if;
  */

          // Determine polynomials with highest power of s equal to one
            (cr,c0,c1) :=
              Blocks.Continuous.Internal.Filter.Utilities.toHighestPowerOne(
              den1, den2);
        end Butterworth;

        function ChebyshevI
            "Return base filter coefficients of Chebyshev I filter (= low pass filter with w_cut = 1 rad/s)"
          import Modelica.Math.asinh;
          import Modelica.Constants.pi;

          extends Modelica.Icons.Function;

          input Integer order(min=1) "Order of filter";
          input Real A_ripple = 0.5 "Pass band ripple in [dB]";
          input Boolean normalized=true
              "= true, if amplitude at f_cut = -3db, otherwise unmodified filter";

          output Real cr[mod(order, 2)] "Coefficient of real pole";
          output Real c0[integer(order/2)]
              "Coefficients of s^0 term if conjugate complex pole";
          output Real c1[integer(order/2)]
              "Coefficients of s^1 term if conjugate complex pole";
          protected
          Real epsilon;
          Real fac;
          Real alpha=1.0 "Frequency correction factor";
          Real alpha2 "= alpha*alpha";
          Real den1[size(cr,1)]
              "[p] coefficients of denominator first order polynomials (a*p + 1)";
          Real den2[size(c0, 1),2]
              "[p^2, p] coefficients of denominator second order polynomials (b*p^2 + a*p + 1)";
        algorithm
            epsilon := sqrt(10^(A_ripple/10) - 1);
            fac := asinh(1/epsilon)/order;

            den1 := fill(1/sinh(fac),size(den1,1));
            if size(cr,1) == 0 then
               for i in 1:size(c0, 1) loop
                  den2[i,1] :=1/(cosh(fac)^2 - cos((2*i - 1)*pi/(2*order))^2);
                  den2[i,2] :=2*den2[i, 1]*sinh(fac)*cos((2*i - 1)*pi/(2*order));
               end for;
            else
               for i in 1:size(c0, 1) loop
                  den2[i,1] :=1/(cosh(fac)^2 - cos(i*pi/order)^2);
                  den2[i,2] :=2*den2[i, 1]*sinh(fac)*cos(i*pi/order);
               end for;
            end if;

            /* Transformation of filter transfer function with "new(p) = alpha*p"
       in order that the filter transfer function has an amplitude of
       -3 db at the cutoff frequency
    */
            if normalized then
              alpha :=
                Blocks.Continuous.Internal.Filter.Utilities.normalizationFactor(
                den1, den2);
              alpha2 := alpha*alpha;
              for i in 1:size(c0, 1) loop
                den2[i, 1] := den2[i, 1]*alpha2;
                den2[i, 2] := den2[i, 2]*alpha;
              end for;
              den1 := den1*alpha;
            end if;

          // Determine polynomials with highest power of s equal to one
            (cr,c0,c1) :=
              Blocks.Continuous.Internal.Filter.Utilities.toHighestPowerOne(
              den1, den2);
        end ChebyshevI;
        end base;

        package coefficients "Filter coefficients"
            extends Modelica.Icons.InternalPackage;
        function lowPass
            "Return low pass filter coefficients at given cut-off frequency"
          import Modelica.Constants.pi;
          extends Modelica.Icons.Function;

          input Real cr_in[:] "Coefficients of real poles";
          input Real c0_in[:]
              "Coefficients of s^0 term if conjugate complex pole";
          input Real c1_in[size(c0_in,1)]
              "Coefficients of s^1 term if conjugate complex pole";
          input SI.Frequency f_cut "Cut-off frequency";

          output Real cr[size(cr_in,1)] "Coefficient of real pole";
          output Real c0[size(c0_in,1)]
              "Coefficients of s^0 term if conjugate complex pole";
          output Real c1[size(c0_in,1)]
              "Coefficients of s^1 term if conjugate complex pole";

          protected
          SI.AngularVelocity w_cut=2*pi*f_cut
              "Cut-off angular frequency";
          Real w_cut2=w_cut*w_cut;

        algorithm
          assert(f_cut > 0, "Cut-off frequency f_cut must be positive");

          /* Change filter coefficients according to transformation new(s) = s/w_cut
     s + cr           -> (s/w) + cr              = (s + w*cr)/w
     s^2 + c1*s + c0  -> (s/w)^2 + c1*(s/w) + c0 = (s^2 + (c1*w)*s + (c0*w^2))/w^2
  */
          cr := w_cut*cr_in;
          c1 := w_cut*c1_in;
          c0 := w_cut2*c0_in;

        end lowPass;

        function highPass
            "Return high pass filter coefficients at given cut-off frequency"
          import Modelica.Constants.pi;
          extends Modelica.Icons.Function;

          input Real cr_in[:] "Coefficients of real poles";
          input Real c0_in[:]
              "Coefficients of s^0 term if conjugate complex pole";
          input Real c1_in[size(c0_in,1)]
              "Coefficients of s^1 term if conjugate complex pole";
          input SI.Frequency f_cut "Cut-off frequency";

          output Real cr[size(cr_in,1)] "Coefficient of real pole";
          output Real c0[size(c0_in,1)]
              "Coefficients of s^0 term if conjugate complex pole";
          output Real c1[size(c0_in,1)]
              "Coefficients of s^1 term if conjugate complex pole";

          protected
          SI.AngularVelocity w_cut=2*pi*f_cut
              "Cut-off angular frequency";
          Real w_cut2=w_cut*w_cut;

        algorithm
          assert(f_cut > 0, "Cut-off frequency f_cut must be positive");

          /* Change filter coefficients according to transformation: new(s) = 1/s
        1/(s + cr)          -> 1/(1/s + cr)                = (1/cr)*s / (s + (1/cr))
        1/(s^2 + c1*s + c0) -> 1/((1/s)^2 + c1*(1/s) + c0) = (1/c0)*s^2 / (s^2 + (c1/c0)*s + 1/c0)

     Check whether transformed roots are also conjugate complex:
        c0 - c1^2/4 > 0  -> (1/c0) - (c1/c0)^2 / 4
                            = (c0 - c1^2/4) / c0^2 > 0
        It is therefore guaranteed that the roots remain conjugate complex

     Change filter coefficients according to transformation new(s) = s/w_cut
        s + 1/cr                -> (s/w) + 1/cr                   = (s + w/cr)/w
        s^2 + (c1/c0)*s + 1/c0  -> (s/w)^2 + (c1/c0)*(s/w) + 1/c0 = (s^2 + (w*c1/c0)*s + (w^2/c0))/w^2
  */
          for i in 1:size(cr_in,1) loop
             cr[i] := w_cut/cr_in[i];
          end for;

          for i in 1:size(c0_in,1) loop
             c0[i] := w_cut2/c0_in[i];
             c1[i] := w_cut*c1_in[i]/c0_in[i];
          end for;

        end highPass;

        function bandPass
            "Return band pass filter coefficients at given cut-off frequency"
          import Modelica.Constants.pi;
          extends Modelica.Icons.Function;

          input Real cr_in[:] "Coefficients of real poles";
          input Real c0_in[:]
              "Coefficients of s^0 term if conjugate complex pole";
          input Real c1_in[size(c0_in,1)]
              "Coefficients of s^1 term if conjugate complex pole";
          input SI.Frequency f_min
              "Band of band pass filter is f_min (A=-3db) .. f_max (A=-3db)";
          input SI.Frequency f_max "Upper band frequency";

          output Real cr[0] "Coefficient of real pole";
          output Real c0[size(cr_in,1) + 2*size(c0_in,1)]
              "Coefficients of s^0 term if conjugate complex pole";
          output Real c1[size(cr_in,1) + 2*size(c0_in,1)]
              "Coefficients of s^1 term if conjugate complex pole";
          output Real cn "Numerator coefficient of the PT2 terms";
          protected
          SI.Frequency f0 = sqrt(f_min*f_max);
          SI.AngularVelocity w_cut=2*pi*f0
              "Cut-off angular frequency";
          Real w_band = (f_max - f_min) / f0;
          Real w_cut2=w_cut*w_cut;
          Real c;
          Real alpha;
          Integer j;
        algorithm
          assert(f_min > 0 and f_min < f_max, "Band frequencies f_min and f_max are wrong");

            /* The band pass filter is derived from the low pass filter by
       the transformation new(s) = (s + 1/s)/w   (w = w_band = (f_max - f_min)/sqrt(f_max*f_min) )

       1/(s + cr)         -> 1/((s/w + 1/s/w) + cr)
                             = w*s / (s^2 + cr*w*s + 1)

       1/(s^2 + c1*s + c0) -> 1/( (s+1/s)^2/w^2 + c1*(s + 1/s)/w + c0 )
                              = 1 /( ( s^2 + 1/s^2 + 2)/w^2 + (s + 1/s)*c1/w + c0 )
                              = w^2*s^2 / (s^4 + 2*s^2 + 1 + (s^3 + s)*c1*w + c0*w^2*s^2)
                              = w^2*s^2 / (s^4 + c1*w*s^3 + (2+c0*w^2)*s^2 + c1*w*s + 1)

                              Assume the following description with PT2:
                              = w^2*s^2 /( (s^2 + s*(c/alpha) + 1/alpha^2)*
                                           (s^2 + s*(c*alpha) + alpha^2) )
                              = w^2*s^2 / ( s^4 + c*(alpha + 1/alpha)*s^3
                                                + (alpha^2 + 1/alpha^2 + c^2)*s^2
                                                + c*(alpha + 1/alpha)*s + 1 )

                              and therefore:
                                c*(alpha + 1/alpha) = c1*w       -> c = c1*w / (alpha + 1/alpha)
                                                                      = c1*w*alpha/(1+alpha^2)
                                alpha^2 + 1/alpha^2 + c^2 = 2+c0*w^2 -> equation to determine alpha
                                alpha^4 + 1 + c1^2*w^2*alpha^4/(1+alpha^2)^2 = (2+c0*w^2)*alpha^2
                                or z = alpha^2
                                z^2 + c^1^2*w^2*z^2/(1+z)^2 - (2+c0*w^2)*z + 1 = 0

     Check whether roots remain conjugate complex
        c0 - (c1/2)^2 > 0:    1/alpha^2 - (c/alpha)^2/4
                              = 1/alpha^2*(1 - c^2/4)    -> not possible to figure this out

     Afterwards, change filter coefficients according to transformation new(s) = s/w_cut
        w_band*s/(s^2 + c1*s + c0)  -> w_band*(s/w)/((s/w)^2 + c1*(s/w) + c0 =
                                       (w_band/w)*s/(s^2 + (c1*w)*s + (c0*w^2))/w^2) =
                                       (w_band*w)*s/(s^2 + (c1*w)*s + (c0*w^2))
    */
            for i in 1:size(cr_in,1) loop
               c1[i] := w_cut*cr_in[i]*w_band;
               c0[i] := w_cut2;
            end for;

            for i in 1:size(c1_in,1) loop
              alpha :=
                Blocks.Continuous.Internal.Filter.Utilities.bandPassAlpha(
                        c1_in[i],
                        c0_in[i],
                        w_band);
               c       := c1_in[i]*w_band / (alpha + 1/alpha);
               j       := size(cr_in,1) + 2*i - 1;
               c1[j]   := w_cut*c/alpha;
               c1[j+1] := w_cut*c*alpha;
               c0[j]   := w_cut2/alpha^2;
               c0[j+1] := w_cut2*alpha^2;
            end for;

            cn :=w_band*w_cut;

        end bandPass;

        function bandStop
            "Return band stop filter coefficients at given cut-off frequency"
          import Modelica.Constants.pi;
          extends Modelica.Icons.Function;

          input Real cr_in[:] "Coefficients of real poles";
          input Real c0_in[:]
              "Coefficients of s^0 term if conjugate complex pole";
          input Real c1_in[size(c0_in,1)]
              "Coefficients of s^1 term if conjugate complex pole";
          input SI.Frequency f_min
              "Band of band stop filter is f_min (A=-3db) .. f_max (A=-3db)";
          input SI.Frequency f_max "Upper band frequency";

          output Real cr[0] "Coefficient of real pole";
          output Real c0[size(cr_in,1) + 2*size(c0_in,1)]
              "Coefficients of s^0 term if conjugate complex pole";
          output Real c1[size(cr_in,1) + 2*size(c0_in,1)]
              "Coefficients of s^1 term if conjugate complex pole";
          protected
          SI.Frequency f0 = sqrt(f_min*f_max);
          SI.AngularVelocity w_cut=2*pi*f0
              "Cut-off angular frequency";
          Real w_band = (f_max - f_min) / f0;
          Real w_cut2=w_cut*w_cut;
          Real c;
          Real ww;
          Real alpha;
          Integer j;
        algorithm
          assert(f_min > 0 and f_min < f_max, "Band frequencies f_min and f_max are wrong");

            /* The band pass filter is derived from the low pass filter by
       the transformation new(s) = (s + 1/s)/w   (w = w_band = (f_max - f_min)/sqrt(f_max*f_min) )

       1/(s + cr)         -> 1/((s/w + 1/s/w) + cr)
                             = w*s / (s^2 + cr*w*s + 1)

       1/(s^2 + c1*s + c0) -> 1/( (s+1/s)^2/w^2 + c1*(s + 1/s)/w + c0 )
                              = 1 /( ( s^2 + 1/s^2 + 2)/w^2 + (s + 1/s)*c1/w + c0 )
                              = w^2*s^2 / (s^4 + 2*s^2 + 1 + (s^3 + s)*c1*w + c0*w^2*s^2)
                              = w^2*s^2 / (s^4 + c1*w*s^3 + (2+c0*w^2)*s^2 + c1*w*s + 1)

                              Assume the following description with PT2:
                              = w^2*s^2 /( (s^2 + s*(c/alpha) + 1/alpha^2)*
                                           (s^2 + s*(c*alpha) + alpha^2) )
                              = w^2*s^2 / ( s^4 + c*(alpha + 1/alpha)*s^3
                                                + (alpha^2 + 1/alpha^2 + c^2)*s^2
                                                + c*(alpha + 1/alpha)*s + 1 )

                              and therefore:
                                c*(alpha + 1/alpha) = c1*w       -> c = c1*w / (alpha + 1/alpha)
                                                                      = c1*w*alpha/(1+alpha^2)
                                alpha^2 + 1/alpha^2 + c^2 = 2+c0*w^2 -> equation to determine alpha
                                alpha^4 + 1 + c1^2*w^2*alpha^4/(1+alpha^2)^2 = (2+c0*w^2)*alpha^2
                                or z = alpha^2
                                z^2 + c^1^2*w^2*z^2/(1+z)^2 - (2+c0*w^2)*z + 1 = 0

       The band stop filter is derived from the low pass filter by
       the transformation new(s) = w/( (s + 1/s) )   (w = w_band = (f_max - f_min)/sqrt(f_max*f_min) )

       cr/(s + cr)         -> 1/(( w/(s + 1/s) ) + cr)
                              = (s^2 + 1) / (s^2 + (w/cr)*s + 1)

       c0/(s^2 + c1*s + c0) -> c0/( w^2/(s + 1/s)^2 + c1*w/(s + 1/s) + c0 )
                               = c0*(s^2 + 1)^2 / (s^4 + c1*w*s^3/c0 + (2+w^2/b)*s^2 + c1*w*s/c0 + 1)

                               Assume the following description with PT2:
                               = c0*(s^2 + 1)^2 / ( (s^2 + s*(c/alpha) + 1/alpha^2)*
                                                    (s^2 + s*(c*alpha) + alpha^2) )
                               = c0*(s^2 + 1)^2 / (  s^4 + c*(alpha + 1/alpha)*s^3
                                                         + (alpha^2 + 1/alpha^2 + c^2)*s^2
                                                         + c*(alpha + 1/alpha)*p + 1 )

                            and therefore:
                              c*(alpha + 1/alpha) = c1*w/b         -> c = c1*w/(c0*(alpha + 1/alpha))
                              alpha^2 + 1/alpha^2 + c^2 = 2+w^2/c0 -> equation to determine alpha
                              alpha^4 + 1 + (c1*w/c0*alpha^2)^2/(1+alpha^2)^2 = (2+w^2/c0)*alpha^2
                              or z = alpha^2
                              z^2 + (c1*w/c0*z)^2/(1+z)^2 - (2+w^2/c0)*z + 1 = 0

                            same as:  ww = w/c0
                              z^2 + (c1*ww*z)^2/(1+z)^2 - (2+c0*ww)*z + 1 = 0  -> same equation as for BandPass

     Afterwards, change filter coefficients according to transformation new(s) = s/w_cut
        c0*(s^2+1)(s^2 + c1*s + c0)  -> c0*((s/w)^2 + 1) / ((s/w)^2 + c1*(s/w) + c0 =
                                        c0/w^2*(s^2 + w^2) / (s^2 + (c1*w)*s + (c0*w^2))/w^2) =
                                        (s^2 + c0*w^2) / (s^2 + (c1*w)*s + (c0*w^2))
    */
            for i in 1:size(cr_in,1) loop
               c1[i] := w_cut*w_band/cr_in[i];
               c0[i] := w_cut2;
            end for;

            for i in 1:size(c1_in,1) loop
               ww      := w_band/c0_in[i];
              alpha :=
                Blocks.Continuous.Internal.Filter.Utilities.bandPassAlpha(
                        c1_in[i],
                        c0_in[i],
                        ww);
               c       := c1_in[i]*ww / (alpha + 1/alpha);
               j       := size(cr_in,1) + 2*i - 1;
               c1[j]   := w_cut*c/alpha;
               c1[j+1] := w_cut*c*alpha;
               c0[j]   := w_cut2/alpha^2;
               c0[j+1] := w_cut2*alpha^2;
            end for;

        end bandStop;
        end coefficients;

        package roots "Filter roots and gain as needed for block implementations"
            extends Modelica.Icons.InternalPackage;
        function lowPass
            "Return low pass filter roots as needed for block for given cut-off frequency"
          extends Modelica.Icons.Function;

          input Real cr_in[:] "Coefficients of real poles of base filter";
          input Real c0_in[:]
              "Coefficients of s^0 term of base filter if conjugate complex pole";
          input Real c1_in[size(c0_in,1)]
              "Coefficients of s^1 term of base filter if conjugate complex pole";
          input SI.Frequency f_cut "Cut-off frequency";

          output Real r[size(cr_in,1)] "Real eigenvalues";
          output Real a[size(c0_in,1)]
              "Real parts of complex conjugate eigenvalues";
          output Real b[size(c0_in,1)]
              "Imaginary parts of complex conjugate eigenvalues";
          output Real ku[size(c0_in,1)] "Input gain";
          protected
          Real c0[size(c0_in,1)];
          Real c1[size(c0_in,1)];
          Real cr[size(cr_in,1)];
        algorithm
          // Get coefficients of low pass filter at f_cut
          (cr, c0, c1) :=coefficients.lowPass(cr_in, c0_in, c1_in, f_cut);

          // Transform coefficients in to root
          for i in 1:size(cr_in,1) loop
            r[i] :=-cr[i];
          end for;

          for i in 1:size(c0_in,1) loop
            a [i] :=-c1[i]/2;
            b [i] :=sqrt(c0[i] - a[i]*a[i]);
            ku[i] :=c0[i]/b[i];
          end for;

          annotation (Documentation(info="<html>

<p>
The goal is to implement the filter in the following form:
</p>

<blockquote><pre>
// real pole:
 der(x) = r*x - r*u
     y  = x

// complex conjugate poles:
der(x1) = a*x1 - b*x2 + ku*u;
der(x2) = b*x1 + a*x2;
     y  = x2;

          ku = (a^2 + b^2)/b
</pre></blockquote>
<p>
This representation has the following transfer function:
</p>
<blockquote><pre>
// real pole:
    s*y = r*y - r*u
  or
    (s-r)*y = -r*u
  or
    y = -r/(s-r)*u

  comparing coefficients with
    y = cr/(s + cr)*u  ->  r = -cr      // r is the real eigenvalue

// complex conjugate poles
    s*x2 =  a*x2 + b*x1
    s*x1 = -b*x2 + a*x1 + ku*u
  or
    (s-a)*x2               = b*x1  ->  x2 = b/(s-a)*x1
    (s + b^2/(s-a) - a)*x1 = ku*u  ->  (s(s-a) + b^2 - a*(s-a))*x1  = ku*(s-a)*u
                                   ->  (s^2 - 2*a*s + a^2 + b^2)*x1 = ku*(s-a)*u
  or
    x1 = ku*(s-a)/(s^2 - 2*a*s + a^2 + b^2)*u
    x2 = b/(s-a)*ku*(s-a)/(s^2 - 2*a*s + a^2 + b^2)*u
       = b*ku/(s^2 - 2*a*s + a^2 + b^2)*u
    y  = x2

  comparing coefficients with
    y = c0/(s^2 + c1*s + c0)*u  ->  a  = -c1/2
                                    b  = sqrt(c0 - a^2)
                                    ku = c0/b
                                       = (a^2 + b^2)/b

  comparing with eigenvalue representation:
    (s - (a+jb))*(s - (a-jb)) = s^2 -2*a*s + a^2 + b^2
  shows that:
    a: real part of eigenvalue
    b: imaginary part of eigenvalue

  time -> infinity:
    y(s=0) = x2(s=0) = 1
             x1(s=0) = -ku*a/(a^2 + b^2)*u
                     = -(a/b)*u
</pre></blockquote>

</html>"));
        end lowPass;

        function highPass
            "Return high pass filter roots as needed for block for given cut-off frequency"
          extends Modelica.Icons.Function;

          input Real cr_in[:] "Coefficients of real poles of base filter";
          input Real c0_in[:]
              "Coefficients of s^0 term of base filter if conjugate complex pole";
          input Real c1_in[size(c0_in,1)]
              "Coefficients of s^1 term of base filter if conjugate complex pole";
          input SI.Frequency f_cut "Cut-off frequency";

          output Real r[size(cr_in,1)] "Real eigenvalues";
          output Real a[size(c0_in,1)]
              "Real parts of complex conjugate eigenvalues";
          output Real b[size(c0_in,1)]
              "Imaginary parts of complex conjugate eigenvalues";
          output Real ku[size(c0_in,1)] "Gains of input terms";
          output Real k1[size(c0_in,1)] "Gains of y = k1*x1 + k2*x + u";
          output Real k2[size(c0_in,1)] "Gains of y = k1*x1 + k2*x + u";
          protected
          Real c0[size(c0_in,1)];
          Real c1[size(c0_in,1)];
          Real cr[size(cr_in,1)];
          Real ba2;
        algorithm
          // Get coefficients of high pass filter at f_cut
          (cr, c0, c1) :=coefficients.highPass(cr_in, c0_in, c1_in, f_cut);

          // Transform coefficients in to roots
          for i in 1:size(cr_in,1) loop
            r[i] :=-cr[i];
          end for;

          for i in 1:size(c0_in,1) loop
            a[i]  := -c1[i]/2;
            b[i]  := sqrt(c0[i] - a[i]*a[i]);
            ku[i] := c0[i]/b[i];
            k1[i] := 2*a[i]/ku[i];
            ba2   := (b[i]/a[i])^2;
            k2[i] := (1-ba2)/(1+ba2);
          end for;

          annotation (Documentation(info="<html>

<p>
The goal is to implement the filter in the following form:
</p>

<blockquote><pre>
// real pole:
 der(x) = r*x - r*u
     y  = -x + u

// complex conjugate poles:
der(x1) = a*x1 - b*x2 + ku*u;
der(x2) = b*x1 + a*x2;
     y  = k1*x1 + k2*x2 + u;

          ku = (a^2 + b^2)/b
          k1 = 2*a/ku
          k2 = (a^2 - b^2) / (b*ku)
             = (a^2 - b^2) / (a^2 + b^2)
             = (1 - (b/a)^2) / (1 + (b/a)^2)
</pre></blockquote>
<p>
This representation has the following transfer function:
</p>
<blockquote><pre>
// real pole:
    s*x = r*x - r*u
  or
    (s-r)*x = -r*u   -> x = -r/(s-r)*u
  or
    y = r/(s-r)*u + (s-r)/(s-r)*u
      = (r+s-r)/(s-r)*u
      = s/(s-r)*u

// comparing coefficients with
    y = s/(s + cr)*u  ->  r = -cr      // r is the real eigenvalue

// complex conjugate poles
    s*x2 =  a*x2 + b*x1
    s*x1 = -b*x2 + a*x1 + ku*u
  or
    (s-a)*x2               = b*x1  ->  x2 = b/(s-a)*x1
    (s + b^2/(s-a) - a)*x1 = ku*u  ->  (s(s-a) + b^2 - a*(s-a))*x1  = ku*(s-a)*u
                                   ->  (s^2 - 2*a*s + a^2 + b^2)*x1 = ku*(s-a)*u
  or
    x1 = ku*(s-a)/(s^2 - 2*a*s + a^2 + b^2)*u
    x2 = b/(s-a)*ku*(s-a)/(s^2 - 2*a*s + a^2 + b^2)*u
       = b*ku/(s^2 - 2*a*s + a^2 + b^2)*u
    y  = k1*x1 + k2*x2 + u
       = (k1*ku*(s-a) + k2*b*ku +  s^2 - 2*a*s + a^2 + b^2) /
         (s^2 - 2*a*s + a^2 + b^2)*u
       = (s^2 + (k1*ku - 2*a)*s + k2*b*ku - k1*ku*a + a^2 + b^2) /
         (s^2 - 2*a*s + a^2 + b^2)*u
       = (s^2 + (2*a-2*a)*s + a^2 - b^2 - 2*a^2 + a^2 + b^2) /
         (s^2 - 2*a*s + a^2 + b^2)*u
       = s^2 / (s^2 - 2*a*s + a^2 + b^2)*u

// comparing coefficients with
    y = s^2/(s^2 + c1*s + c0)*u  ->  a = -c1/2
                                     b = sqrt(c0 - a^2)

// comparing with eigenvalue representation:
    (s - (a+jb))*(s - (a-jb)) = s^2 -2*a*s + a^2 + b^2
// shows that:
//   a: real part of eigenvalue
//   b: imaginary part of eigenvalue
</pre></blockquote>

</html>"));
        end highPass;

        function bandPass
            "Return band pass filter roots as needed for block for given cut-off frequency"
          extends Modelica.Icons.Function;

          input Real cr_in[:] "Coefficients of real poles of base filter";
          input Real c0_in[:]
              "Coefficients of s^0 term of base filter if conjugate complex pole";
          input Real c1_in[size(c0_in,1)]
              "Coefficients of s^1 term of base filter if conjugate complex pole";
          input SI.Frequency f_min
              "Band of band pass filter is f_min (A=-3db) .. f_max (A=-3db)";
          input SI.Frequency f_max "Upper band frequency";

          output Real a[size(cr_in,1) + 2*size(c0_in,1)]
              "Real parts of complex conjugate eigenvalues";
          output Real b[size(cr_in,1) + 2*size(c0_in,1)]
              "Imaginary parts of complex conjugate eigenvalues";
          output Real ku[size(cr_in,1) + 2*size(c0_in,1)] "Gains of input terms";
          output Real k1[size(cr_in,1) + 2*size(c0_in,1)]
              "Gains of y = k1*x1 + k2*x";
          output Real k2[size(cr_in,1) + 2*size(c0_in,1)]
              "Gains of y = k1*x1 + k2*x";
          protected
          Real cr[0];
          Real c0[size(a,1)];
          Real c1[size(a,1)];
          Real cn;
          Real bb;
        algorithm
          // Get coefficients of band pass filter at f_cut
          (cr, c0, c1, cn) :=coefficients.bandPass(cr_in, c0_in, c1_in, f_min, f_max);

          // Transform coefficients in to roots
          for i in 1:size(a,1) loop
            a[i]  := -c1[i]/2;
            bb    := c0[i] - a[i]*a[i];
            assert(bb >= 0, "\nNot possible to use band pass filter, since transformation results in\n"+
                            "system that does not have conjugate complex poles.\n" +
                            "Try to use another analog filter for the band pass.\n");
            b[i]  := sqrt(bb);
            ku[i] := c0[i]/b[i];
            k1[i] := cn/ku[i];
            k2[i] := cn*a[i]/(b[i]*ku[i]);
          end for;

          annotation (Documentation(info="<html>

<p>
The goal is to implement the filter in the following form:
</p>

<blockquote><pre>
// complex conjugate poles:
der(x1) = a*x1 - b*x2 + ku*u;
der(x2) = b*x1 + a*x2;
     y  = k1*x1 + k2*x2;

          ku = (a^2 + b^2)/b
          k1 = cn/ku
          k2 = cn*a/(b*ku)
</pre></blockquote>
<p>
This representation has the following transfer function:
</p>
<blockquote><pre>
// complex conjugate poles
    s*x2 =  a*x2 + b*x1
    s*x1 = -b*x2 + a*x1 + ku*u
  or
    (s-a)*x2               = b*x1  ->  x2 = b/(s-a)*x1
    (s + b^2/(s-a) - a)*x1 = ku*u  ->  (s(s-a) + b^2 - a*(s-a))*x1  = ku*(s-a)*u
                                   ->  (s^2 - 2*a*s + a^2 + b^2)*x1 = ku*(s-a)*u
  or
    x1 = ku*(s-a)/(s^2 - 2*a*s + a^2 + b^2)*u
    x2 = b/(s-a)*ku*(s-a)/(s^2 - 2*a*s + a^2 + b^2)*u
       = b*ku/(s^2 - 2*a*s + a^2 + b^2)*u
    y  = k1*x1 + k2*x2
       = (k1*ku*(s-a) + k2*b*ku) / (s^2 - 2*a*s + a^2 + b^2)*u
       = (k1*ku*s + k2*b*ku - k1*ku*a) / (s^2 - 2*a*s + a^2 + b^2)*u
       = (cn*s + cn*a - cn*a) / (s^2 - 2*a*s + a^2 + b^2)*u
       = cn*s / (s^2 - 2*a*s + a^2 + b^2)*u

  comparing coefficients with
    y = cn*s / (s^2 + c1*s + c0)*u  ->  a = -c1/2
                                        b = sqrt(c0 - a^2)

  comparing with eigenvalue representation:
    (s - (a+jb))*(s - (a-jb)) = s^2 -2*a*s + a^2 + b^2
  shows that:
    a: real part of eigenvalue
    b: imaginary part of eigenvalue
</pre></blockquote>

</html>"));
        end bandPass;

        function bandStop
            "Return band stop filter roots as needed for block for given cut-off frequency"
          extends Modelica.Icons.Function;

          input Real cr_in[:] "Coefficients of real poles of base filter";
          input Real c0_in[:]
              "Coefficients of s^0 term of base filter if conjugate complex pole";
          input Real c1_in[size(c0_in,1)]
              "Coefficients of s^1 term of base filter if conjugate complex pole";
          input SI.Frequency f_min
              "Band of band stop filter is f_min (A=-3db) .. f_max (A=-3db)";
          input SI.Frequency f_max "Upper band frequency";

          output Real a[size(cr_in,1) + 2*size(c0_in,1)]
              "Real parts of complex conjugate eigenvalues";
          output Real b[size(cr_in,1) + 2*size(c0_in,1)]
              "Imaginary parts of complex conjugate eigenvalues";
          output Real ku[size(cr_in,1) + 2*size(c0_in,1)] "Gains of input terms";
          output Real k1[size(cr_in,1) + 2*size(c0_in,1)]
              "Gains of y = k1*x1 + k2*x";
          output Real k2[size(cr_in,1) + 2*size(c0_in,1)]
              "Gains of y = k1*x1 + k2*x";
          protected
          Real cr[0];
          Real c0[size(a,1)];
          Real c1[size(a,1)];
          Real cn;
          Real bb;
        algorithm
          // Get coefficients of band stop filter at f_cut
          (cr, c0, c1) :=coefficients.bandStop(cr_in, c0_in, c1_in, f_min, f_max);

          // Transform coefficients in to roots
          for i in 1:size(a,1) loop
            a[i]  := -c1[i]/2;
            bb    := c0[i] - a[i]*a[i];
            assert(bb >= 0, "\nNot possible to use band stop filter, since transformation results in\n"+
                            "system that does not have conjugate complex poles.\n" +
                            "Try to use another analog filter for the band stop filter.\n");
            b[i]  := sqrt(bb);
            ku[i] := c0[i]/b[i];
            k1[i] := 2*a[i]/ku[i];
            k2[i] := (c0[i] + a[i]^2 - b[i]^2)/(b[i]*ku[i]);
          end for;

          annotation (Documentation(info="<html>

<p>
The goal is to implement the filter in the following form:
</p>

<blockquote><pre>
// complex conjugate poles:
der(x1) = a*x1 - b*x2 + ku*u;
der(x2) = b*x1 + a*x2;
     y  = k1*x1 + k2*x2 + u;

          ku = (a^2 + b^2)/b
          k1 = 2*a/ku
          k2 = (c0 + a^2 - b^2)/(b*ku)
</pre></blockquote>
<p>
This representation has the following transfer function:
</p>
<blockquote><pre>
// complex conjugate poles
    s*x2 =  a*x2 + b*x1
    s*x1 = -b*x2 + a*x1 + ku*u
  or
    (s-a)*x2               = b*x1  ->  x2 = b/(s-a)*x1
    (s + b^2/(s-a) - a)*x1 = ku*u  ->  (s(s-a) + b^2 - a*(s-a))*x1  = ku*(s-a)*u
                                   ->  (s^2 - 2*a*s + a^2 + b^2)*x1 = ku*(s-a)*u
  or
    x1 = ku*(s-a)/(s^2 - 2*a*s + a^2 + b^2)*u
    x2 = b/(s-a)*ku*(s-a)/(s^2 - 2*a*s + a^2 + b^2)*u
       = b*ku/(s^2 - 2*a*s + a^2 + b^2)*u
    y  = k1*x1 + k2*x2 + u
       = (k1*ku*(s-a) + k2*b*ku + s^2 - 2*a*s + a^2 + b^2) / (s^2 - 2*a*s + a^2 + b^2)*u
       = (s^2 + (k1*ku-2*a)*s + k2*b*ku - k1*ku*a + a^2 + b^2) / (s^2 - 2*a*s + a^2 + b^2)*u
       = (s^2 + c0 + a^2 - b^2 - 2*a^2 + a^2 + b^2) / (s^2 - 2*a*s + a^2 + b^2)*u
       = (s^2 + c0) / (s^2 - 2*a*s + a^2 + b^2)*u

  comparing coefficients with
    y = (s^2 + c0) / (s^2 + c1*s + c0)*u  ->  a = -c1/2
                                              b = sqrt(c0 - a^2)

  comparing with eigenvalue representation:
    (s - (a+jb))*(s - (a-jb)) = s^2 -2*a*s + a^2 + b^2
  shows that:
    a: real part of eigenvalue
    b: imaginary part of eigenvalue
</pre></blockquote>

</html>"));
        end bandStop;
        end roots;

        package Utilities "Utility functions for filter computations"
            extends Modelica.Icons.InternalPackage;
          function BesselBaseCoefficients
            "Return coefficients of normalized low pass Bessel filter (= gain at cut-off frequency 1 rad/s is decreased 3dB)"
            extends Modelica.Icons.Function;

            import Modelica.Utilities.Streams;
            input Integer order "Order of filter in the range 1..41";
            output Real c1[mod(order, 2)]
              "[p] coefficients of Bessel denominator polynomials (a*p + 1)";
            output Real c2[integer(order/2),2]
              "[p^2, p] coefficients of Bessel denominator polynomials (b2*p^2 + b1*p + 1)";
            output Real alpha "Normalization factor";
          algorithm
            if order == 1 then
              alpha := 1.002377293007601;
              c1[1] := 0.9976283451109835;
            elseif order == 2 then
              alpha := 0.7356641785819585;
              c2[1, 1] := 0.6159132201783791;
              c2[1, 2] := 1.359315879600889;
            elseif order == 3 then
              alpha := 0.5704770156982642;
              c1[1] := 0.7548574865985343;
              c2[1, 1] := 0.4756958028827457;
              c2[1, 2] := 0.9980615136104388;
            elseif order == 4 then
              alpha := 0.4737978580281427;
              c2[1, 1] := 0.4873729247240677;
              c2[1, 2] := 1.337564170455762;
              c2[2, 1] := 0.3877724315741958;
              c2[2, 2] := 0.7730405590839861;
            elseif order == 5 then
              alpha := 0.4126226974763408;
              c1[1] := 0.6645723262620757;
              c2[1, 1] := 0.4115231900614016;
              c2[1, 2] := 1.138349926728708;
              c2[2, 1] := 0.3234938702877912;
              c2[2, 2] := 0.6205992985771313;
            elseif order == 6 then
              alpha := 0.3705098000736233;
              c2[1, 1] := 0.3874508649098960;
              c2[1, 2] := 1.219740879520741;
              c2[2, 1] := 0.3493298843155746;
              c2[2, 2] := 0.9670265529381365;
              c2[3, 1] := 0.2747419229514599;
              c2[3, 2] := 0.5122165075105700;
            elseif order == 7 then
              alpha := 0.3393452623586350;
              c1[1] := 0.5927147125821412;
              c2[1, 1] := 0.3383379423919174;
              c2[1, 2] := 1.092630816438030;
              c2[2, 1] := 0.3001025788696046;
              c2[2, 2] := 0.8289928256598656;
              c2[3, 1] := 0.2372867471539579;
              c2[3, 2] := 0.4325128641920154;
            elseif order == 8 then
              alpha := 0.3150267393795002;
              c2[1, 1] := 0.3151115975207653;
              c2[1, 2] := 1.109403015460190;
              c2[2, 1] := 0.2969344839572762;
              c2[2, 2] := 0.9737455812222699;
              c2[3, 1] := 0.2612545921889538;
              c2[3, 2] := 0.7190394712068573;
              c2[4, 1] := 0.2080523342974281;
              c2[4, 2] := 0.3721456473047434;
            elseif order == 9 then
              alpha := 0.2953310177184124;
              c1[1] := 0.5377196679501422;
              c2[1, 1] := 0.2824689124281034;
              c2[1, 2] := 1.022646191567475;
              c2[2, 1] := 0.2626824161383468;
              c2[2, 2] := 0.8695626454762596;
              c2[3, 1] := 0.2302781917677917;
              c2[3, 2] := 0.6309047553448520;
              c2[4, 1] := 0.1847991729757028;
              c2[4, 2] := 0.3251978031287202;
            elseif order == 10 then
              alpha := 0.2789426890619463;
              c2[1, 1] := 0.2640769908255582;
              c2[1, 2] := 1.019788132875305;
              c2[2, 1] := 0.2540802639216947;
              c2[2, 2] := 0.9377020417760623;
              c2[3, 1] := 0.2343577229427963;
              c2[3, 2] := 0.7802229808216112;
              c2[4, 1] := 0.2052193139338624;
              c2[4, 2] := 0.5594176813008133;
              c2[5, 1] := 0.1659546953748916;
              c2[5, 2] := 0.2878349616233292;
            elseif order == 11 then
              alpha := 0.2650227766037203;
              c1[1] := 0.4950265498954191;
              c2[1, 1] := 0.2411858478546218;
              c2[1, 2] := 0.9567800996387417;
              c2[2, 1] := 0.2296849355380925;
              c2[2, 2] := 0.8592523717113126;
              c2[3, 1] := 0.2107851705677406;
              c2[3, 2] := 0.7040216048898129;
              c2[4, 1] := 0.1846461385164021;
              c2[4, 2] := 0.5006729207276717;
              c2[5, 1] := 0.1504217970817433;
              c2[5, 2] := 0.2575070491320295;
            elseif order == 12 then
              alpha := 0.2530051198547209;
              c2[1, 1] := 0.2268294941204543;
              c2[1, 2] := 0.9473116570034053;
              c2[2, 1] := 0.2207657387793729;
              c2[2, 2] := 0.8933728946287606;
              c2[3, 1] := 0.2087600700376653;
              c2[3, 2] := 0.7886236252756229;
              c2[4, 1] := 0.1909959101492760;
              c2[4, 2] := 0.6389263649257017;
              c2[5, 1] := 0.1675208146048472;
              c2[5, 2] := 0.4517847275162215;
              c2[6, 1] := 0.1374257286372761;
              c2[6, 2] := 0.2324699157474680;
            elseif order == 13 then
              alpha := 0.2424910397561007;
              c1[1] := 0.4608848369928040;
              c2[1, 1] := 0.2099813050274780;
              c2[1, 2] := 0.8992478823790660;
              c2[2, 1] := 0.2027250423101359;
              c2[2, 2] := 0.8328117484224146;
              c2[3, 1] := 0.1907635894058731;
              c2[3, 2] := 0.7257379204691213;
              c2[4, 1] := 0.1742280397887686;
              c2[4, 2] := 0.5830640944868014;
              c2[5, 1] := 0.1530858190490478;
              c2[5, 2] := 0.4106192089751885;
              c2[6, 1] := 0.1264090712880446;
              c2[6, 2] := 0.2114980230156001;
            elseif order == 14 then
              alpha := 0.2331902368695848;
              c2[1, 1] := 0.1986162311411235;
              c2[1, 2] := 0.8876961808055535;
              c2[2, 1] := 0.1946683341271615;
              c2[2, 2] := 0.8500754229171967;
              c2[3, 1] := 0.1868331332895056;
              c2[3, 2] := 0.7764629313723603;
              c2[4, 1] := 0.1752118757862992;
              c2[4, 2] := 0.6699720402924552;
              c2[5, 1] := 0.1598906457908402;
              c2[5, 2] := 0.5348446712848934;
              c2[6, 1] := 0.1407810153019944;
              c2[6, 2] := 0.3755841316563539;
              c2[7, 1] := 0.1169627966707339;
              c2[7, 2] := 0.1937088226304455;
            elseif order == 15 then
              alpha := 0.2248854870552422;
              c1[1] := 0.4328492272335646;
              c2[1, 1] := 0.1857292591004588;
              c2[1, 2] := 0.8496337061962563;
              c2[2, 1] := 0.1808644178280136;
              c2[2, 2] := 0.8020517898136011;
              c2[3, 1] := 0.1728264404199081;
              c2[3, 2] := 0.7247449729331105;
              c2[4, 1] := 0.1616970125901954;
              c2[4, 2] := 0.6205369315943097;
              c2[5, 1] := 0.1475257264578426;
              c2[5, 2] := 0.4929612162355906;
              c2[6, 1] := 0.1301861023357119;
              c2[6, 2] := 0.3454770708040735;
              c2[7, 1] := 0.1087810777120188;
              c2[7, 2] := 0.1784526655428406;
            elseif order == 16 then
              alpha := 0.2174105053474761;
              c2[1, 1] := 0.1765637967473151;
              c2[1, 2] := 0.8377453068635511;
              c2[2, 1] := 0.1738525357503125;
              c2[2, 2] := 0.8102988957433199;
              c2[3, 1] := 0.1684627004613343;
              c2[3, 2] := 0.7563265923413258;
              c2[4, 1] := 0.1604519074815815;
              c2[4, 2] := 0.6776082294687619;
              c2[5, 1] := 0.1498828607802206;
              c2[5, 2] := 0.5766417034027680;
              c2[6, 1] := 0.1367764717792823;
              c2[6, 2] := 0.4563528264410489;
              c2[7, 1] := 0.1209810465419295;
              c2[7, 2] := 0.3193782657322374;
              c2[8, 1] := 0.1016312648007554;
              c2[8, 2] := 0.1652419227369036;
            elseif order == 17 then
              alpha := 0.2106355148193306;
              c1[1] := 0.4093223608497299;
              c2[1, 1] := 0.1664014345826274;
              c2[1, 2] := 0.8067173752345952;
              c2[2, 1] := 0.1629839591538256;
              c2[2, 2] := 0.7712924931447541;
              c2[3, 1] := 0.1573277802512491;
              c2[3, 2] := 0.7134213666303411;
              c2[4, 1] := 0.1494828185148637;
              c2[4, 2] := 0.6347841731714884;
              c2[5, 1] := 0.1394948812681826;
              c2[5, 2] := 0.5375594414619047;
              c2[6, 1] := 0.1273627583380806;
              c2[6, 2] := 0.4241608926375478;
              c2[7, 1] := 0.1129187258461290;
              c2[7, 2] := 0.2965752009703245;
              c2[8, 1] := 0.9533357359908857e-1;
              c2[8, 2] := 0.1537041700889585;
            elseif order == 18 then
              alpha := 0.2044575288651841;
              c2[1, 1] := 0.1588768571976356;
              c2[1, 2] := 0.7951914263212913;
              c2[2, 1] := 0.1569357024981854;
              c2[2, 2] := 0.7744529690772538;
              c2[3, 1] := 0.1530722206358810;
              c2[3, 2] := 0.7335304425992080;
              c2[4, 1] := 0.1473206710524167;
              c2[4, 2] := 0.6735038935387268;
              c2[5, 1] := 0.1397225420331520;
              c2[5, 2] := 0.5959151542621590;
              c2[6, 1] := 0.1303092459809849;
              c2[6, 2] := 0.5026483447894845;
              c2[7, 1] := 0.1190627367060072;
              c2[7, 2] := 0.3956893824587150;
              c2[8, 1] := 0.1058058030798994;
              c2[8, 2] := 0.2765091830730650;
              c2[9, 1] := 0.8974708108800873e-1;
              c2[9, 2] := 0.1435505288284833;
            elseif order == 19 then
              alpha := 0.1987936248083529;
              c1[1] := 0.3892259966869526;
              c2[1, 1] := 0.1506640012172225;
              c2[1, 2] := 0.7693121733774260;
              c2[2, 1] := 0.1481728062796673;
              c2[2, 2] := 0.7421133586741549;
              c2[3, 1] := 0.1440444668388838;
              c2[3, 2] := 0.6975075386214800;
              c2[4, 1] := 0.1383101628540374;
              c2[4, 2] := 0.6365464378910025;
              c2[5, 1] := 0.1310032283190998;
              c2[5, 2] := 0.5606211948462122;
              c2[6, 1] := 0.1221431166405330;
              c2[6, 2] := 0.4713530424221445;
              c2[7, 1] := 0.1116991161103884;
              c2[7, 2] := 0.3703717538617073;
              c2[8, 1] := 0.9948917351196349e-1;
              c2[8, 2] := 0.2587371155559744;
              c2[9, 1] := 0.8475989238107367e-1;
              c2[9, 2] := 0.1345537894555993;
            elseif order == 20 then
              alpha := 0.1935761760416219;
              c2[1, 1] := 0.1443871348337404;
              c2[1, 2] := 0.7584165598446141;
              c2[2, 1] := 0.1429501891353184;
              c2[2, 2] := 0.7423000962318863;
              c2[3, 1] := 0.1400877384920004;
              c2[3, 2] := 0.7104185332215555;
              c2[4, 1] := 0.1358210369491446;
              c2[4, 2] := 0.6634599783272630;
              c2[5, 1] := 0.1301773703034290;
              c2[5, 2] := 0.6024175491895959;
              c2[6, 1] := 0.1231826501439148;
              c2[6, 2] := 0.5285332736326852;
              c2[7, 1] := 0.1148465498575254;
              c2[7, 2] := 0.4431977385498628;
              c2[8, 1] := 0.1051289462376788;
              c2[8, 2] := 0.3477444062821162;
              c2[9, 1] := 0.9384622797485121e-1;
              c2[9, 2] := 0.2429038300327729;
              c2[10, 1] := 0.8028211612831444e-1;
              c2[10, 2] := 0.1265329974009533;
            elseif order == 21 then
              alpha := 0.1887494014766075;
              c1[1] := 0.3718070668941645;
              c2[1, 1] := 0.1376151928386445;
              c2[1, 2] := 0.7364290859445481;
              c2[2, 1] := 0.1357438914390695;
              c2[2, 2] := 0.7150167318935022;
              c2[3, 1] := 0.1326398453462415;
              c2[3, 2] := 0.6798001808470175;
              c2[4, 1] := 0.1283231214897678;
              c2[4, 2] := 0.6314663440439816;
              c2[5, 1] := 0.1228169159777534;
              c2[5, 2] := 0.5709353626166905;
              c2[6, 1] := 0.1161406100773184;
              c2[6, 2] := 0.4993087153571335;
              c2[7, 1] := 0.1082959649233524;
              c2[7, 2] := 0.4177766148584385;
              c2[8, 1] := 0.9923596957485723e-1;
              c2[8, 2] := 0.3274257287232124;
              c2[9, 1] := 0.8877776108724853e-1;
              c2[9, 2] := 0.2287218166767916;
              c2[10, 1] := 0.7624076527736326e-1;
              c2[10, 2] := 0.1193423971506988;
            elseif order == 22 then
              alpha := 0.1842668221199706;
              c2[1, 1] := 0.1323053462701543;
              c2[1, 2] := 0.7262446126765204;
              c2[2, 1] := 0.1312121721769772;
              c2[2, 2] := 0.7134286088450949;
              c2[3, 1] := 0.1290330911166814;
              c2[3, 2] := 0.6880287870435514;
              c2[4, 1] := 0.1257817990372067;
              c2[4, 2] := 0.6505015800059301;
              c2[5, 1] := 0.1214765261983008;
              c2[5, 2] := 0.6015107185211451;
              c2[6, 1] := 0.1161365140967959;
              c2[6, 2] := 0.5418983553698413;
              c2[7, 1] := 0.1097755171533100;
              c2[7, 2] := 0.4726370779831614;
              c2[8, 1] := 0.1023889478519956;
              c2[8, 2] := 0.3947439506537486;
              c2[9, 1] := 0.9392485861253800e-1;
              c2[9, 2] := 0.3090996703083202;
              c2[10, 1] := 0.8420273775456455e-1;
              c2[10, 2] := 0.2159561978556017;
              c2[11, 1] := 0.7257600023938262e-1;
              c2[11, 2] := 0.1128633732721116;
            elseif order == 23 then
              alpha := 0.1800893554453722;
              c1[1] := 0.3565232673929280;
              c2[1, 1] := 0.1266275171652706;
              c2[1, 2] := 0.7072778066734162;
              c2[2, 1] := 0.1251865227648538;
              c2[2, 2] := 0.6900676345785905;
              c2[3, 1] := 0.1227944815236645;
              c2[3, 2] := 0.6617011100576023;
              c2[4, 1] := 0.1194647013077667;
              c2[4, 2] := 0.6226432315773119;
              c2[5, 1] := 0.1152132989252356;
              c2[5, 2] := 0.5735222810625359;
              c2[6, 1] := 0.1100558598478487;
              c2[6, 2] := 0.5151027978024605;
              c2[7, 1] := 0.1040013558214886;
              c2[7, 2] := 0.4482410942032739;
              c2[8, 1] := 0.9704014176512626e-1;
              c2[8, 2] := 0.3738049984631116;
              c2[9, 1] := 0.8911683905758054e-1;
              c2[9, 2] := 0.2925028692588410;
              c2[10, 1] := 0.8005438265072295e-1;
              c2[10, 2] := 0.2044134600278901;
              c2[11, 1] := 0.6923832296800832e-1;
              c2[11, 2] := 0.1069984887283394;
            elseif order == 24 then
              alpha := 0.1761838665838427;
              c2[1, 1] := 0.1220804912720132;
              c2[1, 2] := 0.6978026874156063;
              c2[2, 1] := 0.1212296762358897;
              c2[2, 2] := 0.6874139794926736;
              c2[3, 1] := 0.1195328372961027;
              c2[3, 2] := 0.6667954259551859;
              c2[4, 1] := 0.1169990987333593;
              c2[4, 2] := 0.6362602049901176;
              c2[5, 1] := 0.1136409040480130;
              c2[5, 2] := 0.5962662188435553;
              c2[6, 1] := 0.1094722001757955;
              c2[6, 2] := 0.5474001634109253;
              c2[7, 1] := 0.1045052832229087;
              c2[7, 2] := 0.4903523180249535;
              c2[8, 1] := 0.9874509806025907e-1;
              c2[8, 2] := 0.4258751523524645;
              c2[9, 1] := 0.9217799943472177e-1;
              c2[9, 2] := 0.3547079765396403;
              c2[10, 1] := 0.8474633796250476e-1;
              c2[10, 2] := 0.2774145482392767;
              c2[11, 1] := 0.7627722381240495e-1;
              c2[11, 2] := 0.1939329108084139;
              c2[12, 1] := 0.6618645465422745e-1;
              c2[12, 2] := 0.1016670147947242;
            elseif order == 25 then
              alpha := 0.1725220521949266;
              c1[1] := 0.3429735385896000;
              c2[1, 1] := 0.1172525033170618;
              c2[1, 2] := 0.6812327932576614;
              c2[2, 1] := 0.1161194585333535;
              c2[2, 2] := 0.6671566071153211;
              c2[3, 1] := 0.1142375145794466;
              c2[3, 2] := 0.6439167855053158;
              c2[4, 1] := 0.1116157454252308;
              c2[4, 2] := 0.6118378416180135;
              c2[5, 1] := 0.1082654809459177;
              c2[5, 2] := 0.5713609763370088;
              c2[6, 1] := 0.1041985674230918;
              c2[6, 2] := 0.5230289949762722;
              c2[7, 1] := 0.9942439308123559e-1;
              c2[7, 2] := 0.4674627926041906;
              c2[8, 1] := 0.9394453593830893e-1;
              c2[8, 2] := 0.4053226688298811;
              c2[9, 1] := 0.8774221237222533e-1;
              c2[9, 2] := 0.3372372276379071;
              c2[10, 1] := 0.8075839512216483e-1;
              c2[10, 2] := 0.2636485508005428;
              c2[11, 1] := 0.7282483286646764e-1;
              c2[11, 2] := 0.1843801345273085;
              c2[12, 1] := 0.6338571166846652e-1;
              c2[12, 2] := 0.9680153764737715e-1;
            elseif order == 26 then
              alpha := 0.1690795702796737;
              c2[1, 1] := 0.1133168695796030;
              c2[1, 2] := 0.6724297955493932;
              c2[2, 1] := 0.1126417845769961;
              c2[2, 2] := 0.6638709519790540;
              c2[3, 1] := 0.1112948749545606;
              c2[3, 2] := 0.6468652038763624;
              c2[4, 1] := 0.1092823986944244;
              c2[4, 2] := 0.6216337070799265;
              c2[5, 1] := 0.1066130386697976;
              c2[5, 2] := 0.5885011413992190;
              c2[6, 1] := 0.1032969057045413;
              c2[6, 2] := 0.5478864278297548;
              c2[7, 1] := 0.9934388184210715e-1;
              c2[7, 2] := 0.5002885306054287;
              c2[8, 1] := 0.9476081523436283e-1;
              c2[8, 2] := 0.4462644847551711;
              c2[9, 1] := 0.8954648464575577e-1;
              c2[9, 2] := 0.3863930785049522;
              c2[10, 1] := 0.8368166847159917e-1;
              c2[10, 2] := 0.3212074592527143;
              c2[11, 1] := 0.7710664731701103e-1;
              c2[11, 2] := 0.2510470347119383;
              c2[12, 1] := 0.6965807988411425e-1;
              c2[12, 2] := 0.1756419294111342;
              c2[13, 1] := 0.6080674930548766e-1;
              c2[13, 2] := 0.9234535279274277e-1;
            elseif order == 27 then
              alpha := 0.1658353543067995;
              c1[1] := 0.3308543720638957;
              c2[1, 1] := 0.1091618578712746;
              c2[1, 2] := 0.6577977071169651;
              c2[2, 1] := 0.1082549561495043;
              c2[2, 2] := 0.6461121666520275;
              c2[3, 1] := 0.1067479247890451;
              c2[3, 2] := 0.6267937760991321;
              c2[4, 1] := 0.1046471079537577;
              c2[4, 2] := 0.6000750116745808;
              c2[5, 1] := 0.1019605976654259;
              c2[5, 2] := 0.5662734183049320;
              c2[6, 1] := 0.9869726954433709e-1;
              c2[6, 2] := 0.5257827234948534;
              c2[7, 1] := 0.9486520934132483e-1;
              c2[7, 2] := 0.4790595019077763;
              c2[8, 1] := 0.9046906518775348e-1;
              c2[8, 2] := 0.4266025862147336;
              c2[9, 1] := 0.8550529998276152e-1;
              c2[9, 2] := 0.3689188223512328;
              c2[10, 1] := 0.7995282239306020e-1;
              c2[10, 2] := 0.3064589322702932;
              c2[11, 1] := 0.7375174596252882e-1;
              c2[11, 2] := 0.2394754504667310;
              c2[12, 1] := 0.6674377263329041e-1;
              c2[12, 2] := 0.1676223546666024;
              c2[13, 1] := 0.5842458027529246e-1;
              c2[13, 2] := 0.8825044329219431e-1;
            elseif order == 28 then
              alpha := 0.1627710671942929;
              c2[1, 1] := 0.1057232656113488;
              c2[1, 2] := 0.6496161226860832;
              c2[2, 1] := 0.1051786825724864;
              c2[2, 2] := 0.6424661279909941;
              c2[3, 1] := 0.1040917964935006;
              c2[3, 2] := 0.6282470268918791;
              c2[4, 1] := 0.1024670101953951;
              c2[4, 2] := 0.6071189030701136;
              c2[5, 1] := 0.1003105109519892;
              c2[5, 2] := 0.5793175191747016;
              c2[6, 1] := 0.9762969425430802e-1;
              c2[6, 2] := 0.5451486608855443;
              c2[7, 1] := 0.9443223803058400e-1;
              c2[7, 2] := 0.5049796971628137;
              c2[8, 1] := 0.9072460982036488e-1;
              c2[8, 2] := 0.4592270546572523;
              c2[9, 1] := 0.8650956423253280e-1;
              c2[9, 2] := 0.4083368605952977;
              c2[10, 1] := 0.8178165740374893e-1;
              c2[10, 2] := 0.3527525188880655;
              c2[11, 1] := 0.7651838885868020e-1;
              c2[11, 2] := 0.2928534570013572;
              c2[12, 1] := 0.7066010532447490e-1;
              c2[12, 2] := 0.2288185204390681;
              c2[13, 1] := 0.6405358596145789e-1;
              c2[13, 2] := 0.1602396172588190;
              c2[14, 1] := 0.5621780070227172e-1;
              c2[14, 2] := 0.8447589564915071e-1;
            elseif order == 29 then
              alpha := 0.1598706626277596;
              c1[1] := 0.3199314513011623;
              c2[1, 1] := 0.1021101032532951;
              c2[1, 2] := 0.6365758882240111;
              c2[2, 1] := 0.1013729819392774;
              c2[2, 2] := 0.6267495975736321;
              c2[3, 1] := 0.1001476175660628;
              c2[3, 2] := 0.6104876178266819;
              c2[4, 1] := 0.9843854640428316e-1;
              c2[4, 2] := 0.5879603139195113;
              c2[5, 1] := 0.9625164534591696e-1;
              c2[5, 2] := 0.5594012291050210;
              c2[6, 1] := 0.9359356960417668e-1;
              c2[6, 2] := 0.5251016150410664;
              c2[7, 1] := 0.9047086748649986e-1;
              c2[7, 2] := 0.4854024475590397;
              c2[8, 1] := 0.8688856407189167e-1;
              c2[8, 2] := 0.4406826457109709;
              c2[9, 1] := 0.8284779224069856e-1;
              c2[9, 2] := 0.3913408089298914;
              c2[10, 1] := 0.7834154620997181e-1;
              c2[10, 2] := 0.3377643999400627;
              c2[11, 1] := 0.7334628941928766e-1;
              c2[11, 2] := 0.2802710651919946;
              c2[12, 1] := 0.6780290487362146e-1;
              c2[12, 2] := 0.2189770008083379;
              c2[13, 1] := 0.6156321231528423e-1;
              c2[13, 2] := 0.1534235999306070;
              c2[14, 1] := 0.5416797446761512e-1;
              c2[14, 2] := 0.8098664736760292e-1;
            elseif order == 30 then
              alpha := 0.1571200296252450;
              c2[1, 1] := 0.9908074847842124e-1;
              c2[1, 2] := 0.6289618807831557;
              c2[2, 1] := 0.9863509708328196e-1;
              c2[2, 2] := 0.6229164525571278;
              c2[3, 1] := 0.9774542692037148e-1;
              c2[3, 2] := 0.6108853364240036;
              c2[4, 1] := 0.9641490581986484e-1;
              c2[4, 2] := 0.5929869253412513;
              c2[5, 1] := 0.9464802912225441e-1;
              c2[5, 2] := 0.5693960175547550;
              c2[6, 1] := 0.9245027206218041e-1;
              c2[6, 2] := 0.5403402396359503;
              c2[7, 1] := 0.8982754584112941e-1;
              c2[7, 2] := 0.5060948065875106;
              c2[8, 1] := 0.8678535291732599e-1;
              c2[8, 2] := 0.4669749797983789;
              c2[9, 1] := 0.8332744242052199e-1;
              c2[9, 2] := 0.4233249626334694;
              c2[10, 1] := 0.7945356393775309e-1;
              c2[10, 2] := 0.3755006094498054;
              c2[11, 1] := 0.7515543969833788e-1;
              c2[11, 2] := 0.3238400339292700;
              c2[12, 1] := 0.7040879901685638e-1;
              c2[12, 2] := 0.2686072427439079;
              c2[13, 1] := 0.6515528854010540e-1;
              c2[13, 2] := 0.2098650589782619;
              c2[14, 1] := 0.5925168237177876e-1;
              c2[14, 2] := 0.1471138832654873;
              c2[15, 1] := 0.5225913954211672e-1;
              c2[15, 2] := 0.7775248839507864e-1;
            elseif order == 31 then
              alpha := 0.1545067022920929;
              c1[1] := 0.3100206996451866;
              c2[1, 1] := 0.9591020358831668e-1;
              c2[1, 2] := 0.6172474793293396;
              c2[2, 1] := 0.9530301275601203e-1;
              c2[2, 2] := 0.6088916323460413;
              c2[3, 1] := 0.9429332655402368e-1;
              c2[3, 2] := 0.5950511595503025;
              c2[4, 1] := 0.9288445429894548e-1;
              c2[4, 2] := 0.5758534119053522;
              c2[5, 1] := 0.9108073420087422e-1;
              c2[5, 2] := 0.5514734636081183;
              c2[6, 1] := 0.8888719137536870e-1;
              c2[6, 2] := 0.5221306199481831;
              c2[7, 1] := 0.8630901440239650e-1;
              c2[7, 2] := 0.4880834248148061;
              c2[8, 1] := 0.8335074993373294e-1;
              c2[8, 2] := 0.4496225358496770;
              c2[9, 1] := 0.8001502494376102e-1;
              c2[9, 2] := 0.4070602306679052;
              c2[10, 1] := 0.7630041338037624e-1;
              c2[10, 2] := 0.3607139804818122;
              c2[11, 1] := 0.7219760885744920e-1;
              c2[11, 2] := 0.3108783301229550;
              c2[12, 1] := 0.6768185077153345e-1;
              c2[12, 2] := 0.2577706252514497;
              c2[13, 1] := 0.6269571766328638e-1;
              c2[13, 2] := 0.2014081375889921;
              c2[14, 1] := 0.5710081766945065e-1;
              c2[14, 2] := 0.1412581515841926;
              c2[15, 1] := 0.5047740914807019e-1;
              c2[15, 2] := 0.7474725873250158e-1;
            elseif order == 32 then
              alpha := 0.1520196210848210;
              c2[1, 1] := 0.9322163554339406e-1;
              c2[1, 2] := 0.6101488690506050;
              c2[2, 1] := 0.9285233997694042e-1;
              c2[2, 2] := 0.6049832320721264;
              c2[3, 1] := 0.9211494244473163e-1;
              c2[3, 2] := 0.5946969295569034;
              c2[4, 1] := 0.9101176786042449e-1;
              c2[4, 2] := 0.5793791854364477;
              c2[5, 1] := 0.8954614071360517e-1;
              c2[5, 2] := 0.5591619969234026;
              c2[6, 1] := 0.8772216763680164e-1;
              c2[6, 2] := 0.5342177994699602;
              c2[7, 1] := 0.8554440426912734e-1;
              c2[7, 2] := 0.5047560942986598;
              c2[8, 1] := 0.8301735302045588e-1;
              c2[8, 2] := 0.4710187048140929;
              c2[9, 1] := 0.8014469519188161e-1;
              c2[9, 2] := 0.4332730387207936;
              c2[10, 1] := 0.7692807528893225e-1;
              c2[10, 2] := 0.3918021436411035;
              c2[11, 1] := 0.7336507157284898e-1;
              c2[11, 2] := 0.3468890521471250;
              c2[12, 1] := 0.6944555312763458e-1;
              c2[12, 2] := 0.2987898029050460;
              c2[13, 1] := 0.6514446669420571e-1;
              c2[13, 2] := 0.2476810747407199;
              c2[14, 1] := 0.6040544477732702e-1;
              c2[14, 2] := 0.1935412053397663;
              c2[15, 1] := 0.5509478650672775e-1;
              c2[15, 2] := 0.1358108994174911;
              c2[16, 1] := 0.4881064725720192e-1;
              c2[16, 2] := 0.7194819894416505e-1;
            elseif order == 33 then
              alpha := 0.1496489351138032;
              c1[1] := 0.3009752799176432;
              c2[1, 1] := 0.9041725460994505e-1;
              c2[1, 2] := 0.5995521047364046;
              c2[2, 1] := 0.8991117804113002e-1;
              c2[2, 2] := 0.5923764112099496;
              c2[3, 1] := 0.8906941547422532e-1;
              c2[3, 2] := 0.5804822013853129;
              c2[4, 1] := 0.8789442491445575e-1;
              c2[4, 2] := 0.5639663528946501;
              c2[5, 1] := 0.8638945831033775e-1;
              c2[5, 2] := 0.5429623519607796;
              c2[6, 1] := 0.8455834602616358e-1;
              c2[6, 2] := 0.5176379938389326;
              c2[7, 1] := 0.8240517431382334e-1;
              c2[7, 2] := 0.4881921474066189;
              c2[8, 1] := 0.7993380417355076e-1;
              c2[8, 2] := 0.4548502528082586;
              c2[9, 1] := 0.7714713890732801e-1;
              c2[9, 2] := 0.4178579388038483;
              c2[10, 1] := 0.7404596598181127e-1;
              c2[10, 2] := 0.3774715722484659;
              c2[11, 1] := 0.7062702339160462e-1;
              c2[11, 2] := 0.3339432938810453;
              c2[12, 1] := 0.6687952672391507e-1;
              c2[12, 2] := 0.2874950693388235;
              c2[13, 1] := 0.6277828912909767e-1;
              c2[13, 2] := 0.2382680702894708;
              c2[14, 1] := 0.5826808305383988e-1;
              c2[14, 2] := 0.1862073169968455;
              c2[15, 1] := 0.5321974125363517e-1;
              c2[15, 2] := 0.1307323751236313;
              c2[16, 1] := 0.4724820282032780e-1;
              c2[16, 2] := 0.6933542082177094e-1;
            elseif order == 34 then
              alpha := 0.1473858373968463;
              c2[1, 1] := 0.8801537152275983e-1;
              c2[1, 2] := 0.5929204288972172;
              c2[2, 1] := 0.8770594341007476e-1;
              c2[2, 2] := 0.5884653382247518;
              c2[3, 1] := 0.8708797598072095e-1;
              c2[3, 2] := 0.5795895850253119;
              c2[4, 1] := 0.8616320590689187e-1;
              c2[4, 2] := 0.5663615383647170;
              c2[5, 1] := 0.8493413175570858e-1;
              c2[5, 2] := 0.5488825092350877;
              c2[6, 1] := 0.8340387368687513e-1;
              c2[6, 2] := 0.5272851839324592;
              c2[7, 1] := 0.8157596213131521e-1;
              c2[7, 2] := 0.5017313864372913;
              c2[8, 1] := 0.7945402670834270e-1;
              c2[8, 2] := 0.4724089864574216;
              c2[9, 1] := 0.7704133559556429e-1;
              c2[9, 2] := 0.4395276256463053;
              c2[10, 1] := 0.7434009635219704e-1;
              c2[10, 2] := 0.4033126590648964;
              c2[11, 1] := 0.7135035113853376e-1;
              c2[11, 2] := 0.3639961488919042;
              c2[12, 1] := 0.6806813160738834e-1;
              c2[12, 2] := 0.3218025212900124;
              c2[13, 1] := 0.6448214312000864e-1;
              c2[13, 2] := 0.2769235521088158;
              c2[14, 1] := 0.6056719318430530e-1;
              c2[14, 2] := 0.2294693573271038;
              c2[15, 1] := 0.5626925196925040e-1;
              c2[15, 2] := 0.1793564218840015;
              c2[16, 1] := 0.5146352031547277e-1;
              c2[16, 2] := 0.1259877129326412;
              c2[17, 1] := 0.4578069074410591e-1;
              c2[17, 2] := 0.6689147319568768e-1;
            elseif order == 35 then
              alpha := 0.1452224267615486;
              c1[1] := 0.2926764667564367;
              c2[1, 1] := 0.8551731299267280e-1;
              c2[1, 2] := 0.5832758214629523;
              c2[2, 1] := 0.8509109732853060e-1;
              c2[2, 2] := 0.5770596582643844;
              c2[3, 1] := 0.8438201446671953e-1;
              c2[3, 2] := 0.5667497616665494;
              c2[4, 1] := 0.8339191981579831e-1;
              c2[4, 2] := 0.5524209816238369;
              c2[5, 1] := 0.8212328610083385e-1;
              c2[5, 2] := 0.5341766459916322;
              c2[6, 1] := 0.8057906332198853e-1;
              c2[6, 2] := 0.5121470053512750;
              c2[7, 1] := 0.7876247299954955e-1;
              c2[7, 2] := 0.4864870722254752;
              c2[8, 1] := 0.7667670879950268e-1;
              c2[8, 2] := 0.4573736721705665;
              c2[9, 1] := 0.7432449556218945e-1;
              c2[9, 2] := 0.4250013835198991;
              c2[10, 1] := 0.7170742126011575e-1;
              c2[10, 2] := 0.3895767735915445;
              c2[11, 1] := 0.6882488171701314e-1;
              c2[11, 2] := 0.3513097926737368;
              c2[12, 1] := 0.6567231746957568e-1;
              c2[12, 2] := 0.3103999917596611;
              c2[13, 1] := 0.6223804362223595e-1;
              c2[13, 2] := 0.2670123611280899;
              c2[14, 1] := 0.5849696460782910e-1;
              c2[14, 2] := 0.2212298104867592;
              c2[15, 1] := 0.5439628409499822e-1;
              c2[15, 2] := 0.1729443731341637;
              c2[16, 1] := 0.4981540179136920e-1;
              c2[16, 2] := 0.1215462157134930;
              c2[17, 1] := 0.4439981033536435e-1;
              c2[17, 2] := 0.6460098363520967e-1;
            elseif order == 36 then
              alpha := 0.1431515914458580;
              c2[1, 1] := 0.8335881847130301e-1;
              c2[1, 2] := 0.5770670512160201;
              c2[2, 1] := 0.8309698922852212e-1;
              c2[2, 2] := 0.5731929100172432;
              c2[3, 1] := 0.8257400347039723e-1;
              c2[3, 2] := 0.5654713811993058;
              c2[4, 1] := 0.8179117911600136e-1;
              c2[4, 2] := 0.5539556343603020;
              c2[5, 1] := 0.8075042173126963e-1;
              c2[5, 2] := 0.5387245649546684;
              c2[6, 1] := 0.7945413151258206e-1;
              c2[6, 2] := 0.5198817177723069;
              c2[7, 1] := 0.7790506514288866e-1;
              c2[7, 2] := 0.4975537629595409;
              c2[8, 1] := 0.7610613635339480e-1;
              c2[8, 2] := 0.4718884193866789;
              c2[9, 1] := 0.7406012816626425e-1;
              c2[9, 2] := 0.4430516443136726;
              c2[10, 1] := 0.7176927060205631e-1;
              c2[10, 2] := 0.4112237708115829;
              c2[11, 1] := 0.6923460172504251e-1;
              c2[11, 2] := 0.3765940116389730;
              c2[12, 1] := 0.6645495833489556e-1;
              c2[12, 2] := 0.3393522147815403;
              c2[13, 1] := 0.6342528888937094e-1;
              c2[13, 2] := 0.2996755899575573;
              c2[14, 1] := 0.6013361864949449e-1;
              c2[14, 2] := 0.2577053294053830;
              c2[15, 1] := 0.5655503081322404e-1;
              c2[15, 2] := 0.2135004731531631;
              c2[16, 1] := 0.5263798119559069e-1;
              c2[16, 2] := 0.1669320999865636;
              c2[17, 1] := 0.4826589873626196e-1;
              c2[17, 2] := 0.1173807590715484;
              c2[18, 1] := 0.4309819397289806e-1;
              c2[18, 2] := 0.6245036108880222e-1;
            elseif order == 37 then
              alpha := 0.1411669104782917;
              c1[1] := 0.2850271036215707;
              c2[1, 1] := 0.8111958235023328e-1;
              c2[1, 2] := 0.5682412610563970;
              c2[2, 1] := 0.8075727567979578e-1;
              c2[2, 2] := 0.5628142923227016;
              c2[3, 1] := 0.8015440554413301e-1;
              c2[3, 2] := 0.5538087696879930;
              c2[4, 1] := 0.7931239302677386e-1;
              c2[4, 2] := 0.5412833323304460;
              c2[5, 1] := 0.7823314328639347e-1;
              c2[5, 2] := 0.5253190555393968;
              c2[6, 1] := 0.7691895211595101e-1;
              c2[6, 2] := 0.5060183741977191;
              c2[7, 1] := 0.7537237072011853e-1;
              c2[7, 2] := 0.4835036020049034;
              c2[8, 1] := 0.7359601294804538e-1;
              c2[8, 2] := 0.4579149413954837;
              c2[9, 1] := 0.7159227884849299e-1;
              c2[9, 2] := 0.4294078049978829;
              c2[10, 1] := 0.6936295002846032e-1;
              c2[10, 2] := 0.3981491350382047;
              c2[11, 1] := 0.6690857785828917e-1;
              c2[11, 2] := 0.3643121502867948;
              c2[12, 1] := 0.6422751692085542e-1;
              c2[12, 2] := 0.3280684291406284;
              c2[13, 1] := 0.6131430866206096e-1;
              c2[13, 2] := 0.2895750997170303;
              c2[14, 1] := 0.5815677249570920e-1;
              c2[14, 2] := 0.2489521814805720;
              c2[15, 1] := 0.5473023527947980e-1;
              c2[15, 2] := 0.2062377435955363;
              c2[16, 1] := 0.5098441033167034e-1;
              c2[16, 2] := 0.1612849131645336;
              c2[17, 1] := 0.4680658811093562e-1;
              c2[17, 2] := 0.1134672937045305;
              c2[18, 1] := 0.4186928031694695e-1;
              c2[18, 2] := 0.6042754777339966e-1;
            elseif order == 38 then
              alpha := 0.1392625697140030;
              c2[1, 1] := 0.7916943373658329e-1;
              c2[1, 2] := 0.5624158631591745;
              c2[2, 1] := 0.7894592250257840e-1;
              c2[2, 2] := 0.5590219398777304;
              c2[3, 1] := 0.7849941672384930e-1;
              c2[3, 2] := 0.5522551628416841;
              c2[4, 1] := 0.7783093084875645e-1;
              c2[4, 2] := 0.5421574325808380;
              c2[5, 1] := 0.7694193770482690e-1;
              c2[5, 2] := 0.5287909941093643;
              c2[6, 1] := 0.7583430534712885e-1;
              c2[6, 2] := 0.5122376814029880;
              c2[7, 1] := 0.7451020436122948e-1;
              c2[7, 2] := 0.4925978555548549;
              c2[8, 1] := 0.7297197617673508e-1;
              c2[8, 2] := 0.4699889739625235;
              c2[9, 1] := 0.7122194706992953e-1;
              c2[9, 2] := 0.4445436860615774;
              c2[10, 1] := 0.6926216260386816e-1;
              c2[10, 2] := 0.4164072786327193;
              c2[11, 1] := 0.6709399961255503e-1;
              c2[11, 2] := 0.3857341621868851;
              c2[12, 1] := 0.6471757977022456e-1;
              c2[12, 2] := 0.3526828388476838;
              c2[13, 1] := 0.6213084287116965e-1;
              c2[13, 2] := 0.3174082831364342;
              c2[14, 1] := 0.5932799638550641e-1;
              c2[14, 2] := 0.2800495563550299;
              c2[15, 1] := 0.5629672408524944e-1;
              c2[15, 2] := 0.2407078154782509;
              c2[16, 1] := 0.5301264751544952e-1;
              c2[16, 2] := 0.1994026830553859;
              c2[17, 1] := 0.4942673259817896e-1;
              c2[17, 2] := 0.1559719194038917;
              c2[18, 1] := 0.4542996716979947e-1;
              c2[18, 2] := 0.1097844277878470;
              c2[19, 1] := 0.4070720755433961e-1;
              c2[19, 2] := 0.5852181110523043e-1;
            elseif order == 39 then
              alpha := 0.1374332900196804;
              c1[1] := 0.2779468246419593;
              c2[1, 1] := 0.7715084161825772e-1;
              c2[1, 2] := 0.5543001331300056;
              c2[2, 1] := 0.7684028301163326e-1;
              c2[2, 2] := 0.5495289890712267;
              c2[3, 1] := 0.7632343924866024e-1;
              c2[3, 2] := 0.5416083298429741;
              c2[4, 1] := 0.7560141319808483e-1;
              c2[4, 2] := 0.5305846713929198;
              c2[5, 1] := 0.7467569064745969e-1;
              c2[5, 2] := 0.5165224112570647;
              c2[6, 1] := 0.7354807648551346e-1;
              c2[6, 2] := 0.4995030679271456;
              c2[7, 1] := 0.7222060351121389e-1;
              c2[7, 2] := 0.4796242430956156;
              c2[8, 1] := 0.7069540462458585e-1;
              c2[8, 2] := 0.4569982440368368;
              c2[9, 1] := 0.6897453353492381e-1;
              c2[9, 2] := 0.4317502624832354;
              c2[10, 1] := 0.6705970959388781e-1;
              c2[10, 2] := 0.4040159353969854;
              c2[11, 1] := 0.6495194541066725e-1;
              c2[11, 2] := 0.3739379843169939;
              c2[12, 1] := 0.6265098412417610e-1;
              c2[12, 2] := 0.3416613843816217;
              c2[13, 1] := 0.6015440984955930e-1;
              c2[13, 2] := 0.3073260166338746;
              c2[14, 1] := 0.5745615876877304e-1;
              c2[14, 2] := 0.2710546723961181;
              c2[15, 1] := 0.5454383762391338e-1;
              c2[15, 2] := 0.2329316824061170;
              c2[16, 1] := 0.5139340231935751e-1;
              c2[16, 2] := 0.1929604256043231;
              c2[17, 1] := 0.4795705862458131e-1;
              c2[17, 2] := 0.1509655259246037;
              c2[18, 1] := 0.4412933231935506e-1;
              c2[18, 2] := 0.1063130748962878;
              c2[19, 1] := 0.3960672309405603e-1;
              c2[19, 2] := 0.5672356837211527e-1;
            elseif order == 40 then
              alpha := 0.1356742655825434;
              c2[1, 1] := 0.7538038374294594e-1;
              c2[1, 2] := 0.5488228264329617;
              c2[2, 1] := 0.7518806529402738e-1;
              c2[2, 2] := 0.5458297722483311;
              c2[3, 1] := 0.7480383050347119e-1;
              c2[3, 2] := 0.5398604576730540;
              c2[4, 1] := 0.7422847031965465e-1;
              c2[4, 2] := 0.5309482987446206;
              c2[5, 1] := 0.7346313704205006e-1;
              c2[5, 2] := 0.5191429845322307;
              c2[6, 1] := 0.7250930053201402e-1;
              c2[6, 2] := 0.5045099368431007;
              c2[7, 1] := 0.7136868456879621e-1;
              c2[7, 2] := 0.4871295553902607;
              c2[8, 1] := 0.7004317764946634e-1;
              c2[8, 2] := 0.4670962098860498;
              c2[9, 1] := 0.6853470921527828e-1;
              c2[9, 2] := 0.4445169164956202;
              c2[10, 1] := 0.6684507689945471e-1;
              c2[10, 2] := 0.4195095960479698;
              c2[11, 1] := 0.6497570123412630e-1;
              c2[11, 2] := 0.3922007419030645;
              c2[12, 1] := 0.6292726794917847e-1;
              c2[12, 2] := 0.3627221993494397;
              c2[13, 1] := 0.6069918741663154e-1;
              c2[13, 2] := 0.3312065181294388;
              c2[14, 1] := 0.5828873983769410e-1;
              c2[14, 2] := 0.2977798532686911;
              c2[15, 1] := 0.5568964389813015e-1;
              c2[15, 2] := 0.2625503293999835;
              c2[16, 1] := 0.5288947816690705e-1;
              c2[16, 2] := 0.2255872486520188;
              c2[17, 1] := 0.4986456327645859e-1;
              c2[17, 2] := 0.1868796731919594;
              c2[18, 1] := 0.4656832613054458e-1;
              c2[18, 2] := 0.1462410193532463;
              c2[19, 1] := 0.4289867647614935e-1;
              c2[19, 2] := 0.1030361558710747;
              c2[20, 1] := 0.3856310684054106e-1;
              c2[20, 2] := 0.5502423832293889e-1;
            elseif order == 41 then
              alpha := 0.1339811106984253;
              c1[1] := 0.2713685065531391;
              c2[1, 1] := 0.7355140275160984e-1;
              c2[1, 2] := 0.5413274778282860;
              c2[2, 1] := 0.7328319082267173e-1;
              c2[2, 2] := 0.5371064088294270;
              c2[3, 1] := 0.7283676160772547e-1;
              c2[3, 2] := 0.5300963437270770;
              c2[4, 1] := 0.7221298133014343e-1;
              c2[4, 2] := 0.5203345998371490;
              c2[5, 1] := 0.7141302173623395e-1;
              c2[5, 2] := 0.5078728971879841;
              c2[6, 1] := 0.7043831559982149e-1;
              c2[6, 2] := 0.4927768111819803;
              c2[7, 1] := 0.6929049381827268e-1;
              c2[7, 2] := 0.4751250308594139;
              c2[8, 1] := 0.6797129849758392e-1;
              c2[8, 2] := 0.4550083840638406;
              c2[9, 1] := 0.6648246325101609e-1;
              c2[9, 2] := 0.4325285673076087;
              c2[10, 1] := 0.6482554675958526e-1;
              c2[10, 2] := 0.4077964789091151;
              c2[11, 1] := 0.6300169683004558e-1;
              c2[11, 2] := 0.3809299858742483;
              c2[12, 1] := 0.6101130648543355e-1;
              c2[12, 2] := 0.3520508315700898;
              c2[13, 1] := 0.5885349417435808e-1;
              c2[13, 2] := 0.3212801560701271;
              c2[14, 1] := 0.5652528148656809e-1;
              c2[14, 2] := 0.2887316252774887;
              c2[15, 1] := 0.5402021575818373e-1;
              c2[15, 2] := 0.2545001287790888;
              c2[16, 1] := 0.5132588802608274e-1;
              c2[16, 2] := 0.2186415296842951;
              c2[17, 1] := 0.4841900639702602e-1;
              c2[17, 2] := 0.1811322622296060;
              c2[18, 1] := 0.4525419574485134e-1;
              c2[18, 2] := 0.1417762065404688;
              c2[19, 1] := 0.4173260173087802e-1;
              c2[19, 2] := 0.9993834530966510e-1;
              c2[20, 1] := 0.3757210572966463e-1;
              c2[20, 2] := 0.5341611499960143e-1;
            else
              Streams.error("Input argument order (= " + String(order) +
                ") of Bessel filter is not in the range 1..41");
            end if;

            annotation (Documentation(info="<html><p>The transfer function H(p) of a <em>n</em> 'th order Bessel filter is given by</p>
<blockquote><pre>
        Bn(0)
H(p) = -------
        Bn(p)
</pre></blockquote>
<p>with the denominator polynomial</p>
<blockquote><pre>
         n             n  (2n - k)!       p^k
Bn(p) = sum c_k*p^k = sum ----------- * -------   (1)
        k=0           k=0 (n - k)!k!    2^(n-k)
</pre></blockquote>
<p>and the numerator</p>
<blockquote><pre>
               (2n)!     1
Bn(0) = c_0 = ------- * ---- .                    (2)
                n!      2^n
</pre></blockquote>
<p>Although the coefficients c_k are integer numbers, it is not advisable to use the
polynomials in an unfactorized form because the coefficients are fast growing with order
n (c_0 is approximately 0.3e24 and 0.8e59 for order n=20 and order n=40
respectively).</p>

<p>Therefore, the polynomial Bn(p) is factorized to first and second order polynomials with
real coefficients corresponding to zeros and poles representation that is used in this library.</p>

<p>The function returns the coefficients which resulted from factorization of the normalized transfer function</p>
<blockquote><pre>
H'(p') = H(p),  p' = p/w0
</pre></blockquote>
<p>as well as</p>
<blockquote><pre>
alpha = 1/w0
</pre></blockquote>
<p>the reciprocal of the cut of frequency w0 where the gain of the transfer function is
decreased 3dB.</p>

<p>Both, coefficients and cut off frequency were calculated symbolically and were eventually evaluated
with high precision calculation. The results were stored in this function as real
numbers.</p>

<h4>Calculation of normalized Bessel filter coefficients</h4>
<p>Equation</p>
<blockquote><pre>
abs(H(j*w0)) = abs(Bn(0)/Bn(j*w0)) = 10^(-3/20)
</pre></blockquote>
<p>which must be fulfilled for cut off frequency w = w0 leads to</p>
<blockquote><pre>
[Re(Bn(j*w0))]^2 + [Im(Bn(j*w0))]^2 - (Bn(0)^2)*10^(3/10) = 0
</pre></blockquote>
<p>which has exactly one real solution w0 for each order n. This solutions of w0 are
calculated symbolically first and evaluated by using high precise values of the
coefficients c_k calculated by following (1) and (2).</p>

<p>With w0, the coefficients of the factorized polynomial can be computed by calculating the
zeros of the denominator polynomial</p>
<blockquote><pre>
        n
Bn(p) = sum w0^k*c_k*(p/w0)^k
        k=0
</pre></blockquote>
<p>of the normalized transfer function H'(p'). There exist n/2 of conjugate complex
pairs of zeros (beta +-j*gamma) if n is even and one additional real zero (alpha) if n is
odd. Finally, the coefficients a, b1_k, b2_k of the polynomials</p>
<blockquote><pre>
a*p + 1,  n is odd
</pre></blockquote>
<p>and</p>
<blockquote><pre>
b2_k*p^2 + b1_k*p + 1,   k = 1,... div(n,2)
</pre></blockquote>
<p>results from</p>
<blockquote><pre>
a = -1/alpha
</pre></blockquote>
<p>and</p>
<blockquote><pre>
b2_k = 1/(beta_k^2 + gamma_k^2) b1_k = -2*beta_k/(beta_k^2 + gamma_k^2)
</pre></blockquote>
</html>"));
          end BesselBaseCoefficients;

          function toHighestPowerOne
            "Transform filter to form with highest power of s equal 1"
            extends Modelica.Icons.Function;

            input Real den1[:] "[s] coefficients of polynomials (den1[i]*s + 1)";
            input Real den2[:,2]
              "[s^2, s] coefficients of polynomials (den2[i,1]*s^2 + den2[i,2]*s + 1)";
            output Real cr[size(den1, 1)]
              "[s^0] coefficients of polynomials cr[i]*(s+1/cr[i])";
            output Real c0[size(den2, 1)]
              "[s^0] coefficients of polynomials (s^2 + (den2[i,2]/den2[i,1])*s + (1/den2[i,1]))";
            output Real c1[size(den2, 1)]
              "[s^1] coefficients of polynomials (s^2 + (den2[i,2]/den2[i,1])*s + (1/den2[i,1]))";
          algorithm
            for i in 1:size(den1, 1) loop
              cr[i] := 1/den1[i];
            end for;

            for i in 1:size(den2, 1) loop
              c1[i] := den2[i, 2]/den2[i, 1];
              c0[i] := 1/den2[i, 1];
            end for;
          end toHighestPowerOne;

          function normalizationFactor
            "Compute correction factor of low pass filter such that amplitude at cut-off frequency is -3db (=10^(-3/20) = 0.70794...)"
            extends Modelica.Icons.Function;

            import Modelica.Utilities.Streams;

            input Real c1[:]
              "[p] coefficients of denominator polynomials (c1[i}*p + 1)";
            input Real c2[:,2]
              "[p^2, p] coefficients of denominator polynomials (c2[i,1]*p^2 + c2[i,2]*p + 1)";
            output Real alpha "Correction factor (replace p by alpha*p)";
          protected
            Real alpha_min;
            Real alpha_max;

            function normalizationResidue
              "Residue of correction factor computation"
              extends Modelica.Icons.Function;
              input Real c1[:]
                "[p] coefficients of denominator polynomials (c1[i]*p + 1)";
              input Real c2[:,2]
                "[p^2, p] coefficients of denominator polynomials (c2[i,1]*p^2 + c2[i,2]*p + 1)";
              input Real alpha;
              output Real residue;
            protected
              constant Real beta= 10^(-3/20)
                "Amplitude of -3db required, i.e., -3db = 20*log(beta)";
              Real cc1;
              Real cc2;
              Real p;
              Real alpha2=alpha*alpha;
              Real alpha4=alpha2*alpha2;
              Real A2=1.0;
            algorithm
              assert(size(c1,1) <= 1, "Internal error 2 (should not occur)");
              if size(c1, 1) == 1 then
                cc1 := c1[1]*c1[1];
                p := 1 + cc1*alpha2;
                A2 := A2*p;
              end if;
              for i in 1:size(c2, 1) loop
                cc1 := c2[i, 2]*c2[i, 2] - 2*c2[i, 1];
                cc2 := c2[i, 1]*c2[i, 1];
                p := 1 + cc1*alpha2 + cc2*alpha4;
                A2 := A2*p;
              end for;
              residue := 1/sqrt(A2) - beta;
            end normalizationResidue;

            function findInterval "Find interval for the root"
              extends Modelica.Icons.Function;
              input Real c1[:]
                "[p] coefficients of denominator polynomials (a*p + 1)";
              input Real c2[:,2]
                "[p^2, p] coefficients of denominator polynomials (b*p^2 + a*p + 1)";
              output Real alpha_min;
              output Real alpha_max;
            protected
              Real alpha = 1.0;
              Real residue;
            algorithm
              alpha_min :=0;
              residue := normalizationResidue(c1, c2, alpha);
              if residue < 0 then
                 alpha_max :=alpha;
              else
                 while residue >= 0 loop
                    alpha := 1.1*alpha;
                    residue := normalizationResidue(c1, c2, alpha);
                 end while;
                 alpha_max :=alpha;
              end if;
            end findInterval;

          function solveOneNonlinearEquation
              "Solve f(u) = 0; f(u_min) and f(u_max) must have different signs"
              extends Modelica.Icons.Function;
              import Modelica.Utilities.Streams.error;

            input Real c1[:]
                "[p] coefficients of denominator polynomials (c1[i]*p + 1)";
            input Real c2[:,2]
                "[p^2, p] coefficients of denominator polynomials (c2[i,1]*p^2 + c2[i,2]*p + 1)";
            input Real u_min "Lower bound of search interval";
            input Real u_max "Upper bound of search interval";
            input Real tolerance=100*Modelica.Constants.eps
                "Relative tolerance of solution u";
            output Real u "Value of independent variable so that f(u) = 0";

            protected
            constant Real eps=Modelica.Constants.eps "Machine epsilon";
            Real a=u_min "Current best minimum interval value";
            Real b=u_max "Current best maximum interval value";
            Real c "Intermediate point a <= c <= b";
            Real d;
            Real e "b - a";
            Real m;
            Real s;
            Real p;
            Real q;
            Real r;
            Real tol;
            Real fa "= f(a)";
            Real fb "= f(b)";
            Real fc;
            Boolean found=false;
          algorithm
            // Check that f(u_min) and f(u_max) have different sign
            fa := normalizationResidue(c1,c2,u_min);
            fb := normalizationResidue(c1,c2,u_max);
            fc := fb;
            if fa > 0.0 and fb > 0.0 or fa < 0.0 and fb < 0.0 then
              error(
                "The arguments u_min and u_max to solveOneNonlinearEquation(..)\n" +
                "do not bracket the root of the single non-linear equation:\n" +
                "  u_min  = " + String(u_min) + "\n" + "  u_max  = " + String(u_max)
                 + "\n" + "  fa = f(u_min) = " + String(fa) + "\n" +
                "  fb = f(u_max) = " + String(fb) + "\n" +
                "fa and fb must have opposite sign which is not the case");
            end if;

            // Initialize variables
            c := a;
            fc := fa;
            e := b - a;
            d := e;

            // Search loop
            while not found loop
              if abs(fc) < abs(fb) then
                a := b;
                b := c;
                c := a;
                fa := fb;
                fb := fc;
                fc := fa;
              end if;

              tol := 2*eps*abs(b) + tolerance;
              m := (c - b)/2;

              if abs(m) <= tol or fb == 0.0 then
                // root found (interval is small enough)
                found := true;
                u := b;
              else
                // Determine if a bisection is needed
                if abs(e) < tol or abs(fa) <= abs(fb) then
                  e := m;
                  d := e;
                else
                  s := fb/fa;
                  if a == c then
                    // linear interpolation
                    p := 2*m*s;
                    q := 1 - s;
                  else
                    // inverse quadratic interpolation
                    q := fa/fc;
                    r := fb/fc;
                    p := s*(2*m*q*(q - r) - (b - a)*(r - 1));
                    q := (q - 1)*(r - 1)*(s - 1);
                  end if;

                  if p > 0 then
                    q := -q;
                  else
                    p := -p;
                  end if;

                  s := e;
                  e := d;
                  if 2*p < 3*m*q - abs(tol*q) and p < abs(0.5*s*q) then
                    // interpolation successful
                    d := p/q;
                  else
                    // use bi-section
                    e := m;
                    d := e;
                  end if;
                end if;

                // Best guess value is defined as "a"
                a := b;
                fa := fb;
                b := b + (if abs(d) > tol then d else if m > 0 then tol else -tol);
                fb := normalizationResidue(c1,c2,b);

                if fb > 0 and fc > 0 or fb < 0 and fc < 0 then
                  // initialize variables
                  c := a;
                  fc := fa;
                  e := b - a;
                  d := e;
                end if;
              end if;
            end while;

            annotation (Documentation(info="<html>

<p>
This function determines the solution of <strong>one non-linear algebraic equation</strong> \"y=f(u)\"
in <strong>one unknown</strong> \"u\" in a reliable way. It is one of the best numerical
algorithms for this purpose. As input, the nonlinear function f(u)
has to be given, as well as an interval u_min, u_max that
contains the solution, i.e., \"f(u_min)\" and \"f(u_max)\" must
have a different sign. If possible, a smaller interval is computed by
inverse quadratic interpolation (interpolating with a quadratic polynomial
through the last 3 points and computing the zero). If this fails,
bisection is used, which always reduces the interval by a factor of 2.
The inverse quadratic interpolation method has superlinear convergence.
This is roughly the same convergence rate as a globally convergent Newton
method, but without the need to compute derivatives of the non-linear
function. The solver function is a direct mapping of the Algol 60 procedure
\"zero\" to Modelica, from:
</p>

<dl>
<dt> Brent R.P.:</dt>
<dd> <strong>Algorithms for Minimization without derivatives</strong>.
     Prentice Hall, 1973, pp. 58-59.</dd>
</dl>

</html>"));
          end solveOneNonlinearEquation;

          algorithm
             // Find interval for alpha
             (alpha_min, alpha_max) :=findInterval(c1, c2);

             // Compute alpha, so that abs(G(p)) = -3db
             alpha :=solveOneNonlinearEquation(
              c1,
              c2,
              alpha_min,
              alpha_max);
          end normalizationFactor;

          encapsulated function bandPassAlpha "Return alpha for band pass"
            extends Modelica.Icons.Function;

            import Modelica;
             input Real a "Coefficient of s^1";
             input Real b "Coefficient of s^0";
             input Modelica.Units.SI.AngularVelocity w
              "Bandwidth angular frequency";
             output Real alpha "Alpha factor to build up band pass";

          protected
            Real alpha_min;
            Real alpha_max;
            Real z_min;
            Real z_max;
            Real z;

            function residue "Residue of non-linear equation"
              extends Modelica.Icons.Function;
              input Real a;
              input Real b;
              input Real w;
              input Real z;
              output Real res;
            algorithm
              res := z^2 + (a*w*z/(1+z))^2 - (2+b*w^2)*z + 1;
            end residue;

          function solveOneNonlinearEquation
              "Solve f(u) = 0; f(u_min) and f(u_max) must have different signs"
              extends Modelica.Icons.Function;
              import Modelica.Utilities.Streams.error;

            input Real aa;
            input Real bb;
            input Real ww;
            input Real u_min "Lower bound of search interval";
            input Real u_max "Upper bound of search interval";
            input Real tolerance=100*Modelica.Constants.eps
                "Relative tolerance of solution u";
            output Real u "Value of independent variable so that f(u) = 0";

            protected
            constant Real eps=Modelica.Constants.eps "Machine epsilon";
            Real a=u_min "Current best minimum interval value";
            Real b=u_max "Current best maximum interval value";
            Real c "Intermediate point a <= c <= b";
            Real d;
            Real e "b - a";
            Real m;
            Real s;
            Real p;
            Real q;
            Real r;
            Real tol;
            Real fa "= f(a)";
            Real fb "= f(b)";
            Real fc;
            Boolean found=false;
          algorithm
            // Check that f(u_min) and f(u_max) have different sign
            fa := residue(aa,bb,ww,u_min);
            fb := residue(aa,bb,ww,u_max);
            fc := fb;
            if fa > 0.0 and fb > 0.0 or fa < 0.0 and fb < 0.0 then
              error(
                "The arguments u_min and u_max to solveOneNonlinearEquation(..)\n" +
                "do not bracket the root of the single non-linear equation:\n" +
                "  u_min  = " + String(u_min) + "\n" + "  u_max  = " + String(u_max)
                 + "\n" + "  fa = f(u_min) = " + String(fa) + "\n" +
                "  fb = f(u_max) = " + String(fb) + "\n" +
                "fa and fb must have opposite sign which is not the case");
            end if;

            // Initialize variables
            c := a;
            fc := fa;
            e := b - a;
            d := e;

            // Search loop
            while not found loop
              if abs(fc) < abs(fb) then
                a := b;
                b := c;
                c := a;
                fa := fb;
                fb := fc;
                fc := fa;
              end if;

              tol := 2*eps*abs(b) + tolerance;
              m := (c - b)/2;

              if abs(m) <= tol or fb == 0.0 then
                // root found (interval is small enough)
                found := true;
                u := b;
              else
                // Determine if a bisection is needed
                if abs(e) < tol or abs(fa) <= abs(fb) then
                  e := m;
                  d := e;
                else
                  s := fb/fa;
                  if a == c then
                    // linear interpolation
                    p := 2*m*s;
                    q := 1 - s;
                  else
                    // inverse quadratic interpolation
                    q := fa/fc;
                    r := fb/fc;
                    p := s*(2*m*q*(q - r) - (b - a)*(r - 1));
                    q := (q - 1)*(r - 1)*(s - 1);
                  end if;

                  if p > 0 then
                    q := -q;
                  else
                    p := -p;
                  end if;

                  s := e;
                  e := d;
                  if 2*p < 3*m*q - abs(tol*q) and p < abs(0.5*s*q) then
                    // interpolation successful
                    d := p/q;
                  else
                    // use bi-section
                    e := m;
                    d := e;
                  end if;
                end if;

                // Best guess value is defined as "a"
                a := b;
                fa := fb;
                b := b + (if abs(d) > tol then d else if m > 0 then tol else -tol);
                fb := residue(aa,bb,ww,b);

                if fb > 0 and fc > 0 or fb < 0 and fc < 0 then
                  // initialize variables
                  c := a;
                  fc := fa;
                  e := b - a;
                  d := e;
                end if;
              end if;
            end while;

            annotation (Documentation(info="<html>

<p>
This function determines the solution of <strong>one non-linear algebraic equation</strong> \"y=f(u)\"
in <strong>one unknown</strong> \"u\" in a reliable way. It is one of the best numerical
algorithms for this purpose. As input, the nonlinear function f(u)
has to be given, as well as an interval u_min, u_max that
contains the solution, i.e., \"f(u_min)\" and \"f(u_max)\" must
have a different sign. If possible, a smaller interval is computed by
inverse quadratic interpolation (interpolating with a quadratic polynomial
through the last 3 points and computing the zero). If this fails,
bisection is used, which always reduces the interval by a factor of 2.
The inverse quadratic interpolation method has superlinear convergence.
This is roughly the same convergence rate as a globally convergent Newton
method, but without the need to compute derivatives of the non-linear
function. The solver function is a direct mapping of the Algol 60 procedure
\"zero\" to Modelica, from:
</p>

<dl>
<dt> Brent R.P.:</dt>
<dd> <strong>Algorithms for Minimization without derivatives</strong>.
     Prentice Hall, 1973, pp. 58-59.</dd>
</dl>

</html>"));
          end solveOneNonlinearEquation;

          algorithm
            assert( a^2/4 - b <= 0,  "Band pass transformation cannot be computed");
            z :=solveOneNonlinearEquation(a, b, w, 0, 1);
            alpha := sqrt(z);

            annotation (Documentation(info="<html>
<p>
A band pass with bandwidth \"w\" is determined from a low pass
</p>

<blockquote><pre>
1/(p^2 + a*p + b)
</pre></blockquote>

<p>
with the transformation
</p>

<blockquote><pre>
new(p) = (p + 1/p)/w
</pre></blockquote>

<p>
This results in the following derivation:
</p>

<blockquote><pre>
1/(p^2 + a*p + b) -> 1/( (p+1/p)^2/w^2 + a*(p + 1/p)/w + b )
                   = 1 /( ( p^2 + 1/p^2 + 2)/w^2 + (p + 1/p)*a/w + b )
                   = w^2*p^2 / (p^4 + 2*p^2 + 1 + (p^3 + p)a*w + b*w^2*p^2)
                   = w^2*p^2 / (p^4 + a*w*p^3 + (2+b*w^2)*p^2 + a*w*p + 1)
</pre></blockquote>

<p>
This 4th order transfer function shall be split in to two transfer functions of order 2 each
for numerical reasons. With the following formulation, the fourth order
polynomial can be represented (with the unknowns \"c\" and \"alpha\"):
</p>

<blockquote><pre>
g(p) = w^2*p^2 / ( (p*alpha)^2 + c*(p*alpha) + 1) * ( (p/alpha)^2 + c*(p/alpha) + 1)
     = w^2*p^2 / ( p^4 + c*(alpha + 1/alpha)*p^3 + (alpha^2 + 1/alpha^2 + c^2)*p^2
                                                 + c*(alpha + 1/alpha)*p + 1 )
</pre></blockquote>

<p>
Comparison of coefficients:
</p>

<blockquote><pre>
c*(alpha + 1/alpha) = a*w           -> c = a*w / (alpha + 1/alpha)
alpha^2 + 1/alpha^2 + c^2 = 2+b*w^2 -> equation to determine alpha

alpha^4 + 1 + a^2*w^2*alpha^4/(1+alpha^2)^2 = (2+b*w^2)*alpha^2
  or z = alpha^2
z^2 + a^2*w^2*z^2/(1+z)^2 - (2+b*w^2)*z + 1 = 0
</pre></blockquote>

<p>
Therefore the last equation has to be solved for \"z\" (basically, this means to compute
a real zero of a fourth order polynomial):
</p>

<blockquote><pre>
solve: 0 = f(z)  = z^2 + a^2*w^2*z^2/(1+z)^2 - (2+b*w^2)*z + 1  for \"z\"
           f(0)  = 1  &gt; 0
           f(1)  = 1 + a^2*w^2/4 - (2+b*w^2) + 1
                 = (a^2/4 - b)*w^2  &lt; 0
                 // since b - a^2/4 > 0 requirement for complex conjugate poles
-> 0 &lt; z &lt; 1
</pre></blockquote>

<p>
This function computes the solution of this equation and returns \"alpha = sqrt(z)\";
</p>

</html>"));
          end bandPassAlpha;
        end Utilities;
      end Filter;
    end Internal;
    annotation (
      Documentation(info="<html>
<p>
This package contains basic <strong>continuous</strong> input/output blocks
described by differential equations.
</p>

<p>
All blocks of this package can be initialized in different
ways controlled by parameter <strong>initType</strong>. The possible
values of initType are defined in
<a href=\"modelica://Modelica.Blocks.Types.Init\">Modelica.Blocks.Types.Init</a>:
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <tr><td><strong>Name</strong></td>
      <td><strong>Description</strong></td></tr>

  <tr><td><strong>Init.NoInit</strong></td>
      <td>no initialization (start values are used as guess values with fixed=false)</td></tr>

  <tr><td><strong>Init.SteadyState</strong></td>
      <td>steady state initialization (derivatives of states are zero)</td></tr>

  <tr><td><strong>Init.InitialState</strong></td>
      <td>Initialization with initial states</td></tr>

  <tr><td><strong>Init.InitialOutput</strong></td>
      <td>Initialization with initial outputs (and steady state of the states if possible)</td></tr>
</table>

<p>
For backward compatibility reasons the default of all blocks is
<strong>Init.NoInit</strong>, with the exception of Integrator and LimIntegrator
where the default is <strong>Init.InitialState</strong> (this was the initialization
defined in version 2.2 of the Modelica standard library).
</p>

<p>
In many cases, the most useful initial condition is
<strong>Init.SteadyState</strong> because initial transients are then no longer
present. The drawback is that in combination with a non-linear
plant, non-linear algebraic equations occur that might be
difficult to solve if appropriate guess values for the
iteration variables are not provided (i.e., start values with fixed=false).
However, it is often already useful to just initialize
the linear blocks from the Continuous blocks library in SteadyState.
This is uncritical, because only linear algebraic equations occur.
If Init.NoInit is set, then the start values for the states are
interpreted as <strong>guess</strong> values and are propagated to the
states with fixed=<strong>false</strong>.
</p>

<p>
Note, initialization with Init.SteadyState is usually difficult
for a block that contains an integrator
(Integrator, LimIntegrator, PI, PID, LimPID).
This is due to the basic equation of an integrator:
</p>

<blockquote><pre>
<strong>initial equation</strong>
   <strong>der</strong>(y) = 0;   // Init.SteadyState
<strong>equation</strong>
   <strong>der</strong>(y) = k*u;
</pre></blockquote>

<p>
The steady state equation leads to the condition that the input to the
integrator is zero. If the input u is already (directly or indirectly) defined
by another initial condition, then the initialization problem is <strong>singular</strong>
(has none or infinitely many solutions). This situation occurs often
for mechanical systems, where, e.g., u = desiredSpeed - measuredSpeed and
since speed is both a state and a derivative, it is always defined by
Init.InitialState or Init.SteadyState initialization.
</p>

<p>
In such a case, <strong>Init.NoInit</strong> has to be selected for the integrator
and an additional initial equation has to be added to the system
to which the integrator is connected. E.g., useful initial conditions
for a 1-dim. rotational inertia controlled by a PI controller are that
<strong>angle</strong>, <strong>speed</strong>, and <strong>acceleration</strong> of the inertia are zero.
</p>

</html>"),   Icon(graphics={Line(
            origin={0.061,4.184},
            points={{81.939,36.056},{65.362,36.056},{14.39,-26.199},{-29.966,
                113.485},{-65.374,-61.217},{-78.061,-78.184}},
            color={95,95,95},
            smooth=Smooth.Bezier)}));
  end Continuous;

  package Discrete
    "Library of discrete input/output blocks with fixed sample period"

    extends Modelica.Icons.Package;

    block Sampler "Ideal sampling of continuous signals"
      extends Interfaces.DiscreteSISO;

    equation
      when {sampleTrigger, initial()} then
        y = u;
      end when;
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Ellipse(lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{25.0,-10.0},{45.0,10.0}}),
          Line(points={{-100.0,0.0},{-45.0,0.0}},
            color={0,0,127}),
          Line(points={{45.0,0.0},{100.0,0.0}},
            color={0,0,127}),
          Line(points={{-35.0,0.0},{30.0,35.0}},
            color={0,0,127}),
          Ellipse(lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{-45.0,-10.0},{-25.0,10.0}})}),
        Documentation(info="<html>
<p>
Samples the continues input signal with a sampling rate defined
via parameter <strong>samplePeriod</strong>.
</p>
</html>"));
    end Sampler;

    block ZeroOrderHold "Zero order hold of a sampled-data system"
      extends Interfaces.DiscreteSISO;
      output Real ySample(start=0, fixed=true);

    equation
      when {sampleTrigger, initial()} then
        ySample = u;
      end when;
      /* Define y=ySample with an infinitesimal delay to break potential
       algebraic loops if both the continuous and the discrete part have
       direct feedthrough
    */
      y = pre(ySample);
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Line(points={{-78.0,-42.0},{-52.0,-42.0},{-52.0,0.0},{-26.0,0.0},{-26.0,24.0},{-6.0,24.0},{-6.0,64.0},{18.0,64.0},{18.0,20.0},{38.0,20.0},{38.0,0.0},{44.0,0.0},{44.0,0.0},{62.0,0.0}},
            color={0,0,127})}),
        Documentation(info="<html>
<p>
The output is identical to the sampled input signal at sample
time instants and holds the output at the value of the last
sample instant during the sample points.
</p>
</html>"));
    end ZeroOrderHold;

    block FirstOrderHold "First order hold of a sampled-data system"
      extends Blocks.Interfaces.DiscreteSISO;
    protected
      SI.Time tSample;
      Real uSample;
      Real pre_uSample;
      Real c;
    initial equation
      pre(tSample) = time;
      pre(uSample) = u;
      pre(pre_uSample) = u;
      pre(c) = 0.0;
    equation
      when sampleTrigger then
        tSample = time;
        uSample = u;
        pre_uSample = pre(uSample);
        c = if firstTrigger then 0 else (uSample - pre_uSample)/samplePeriod;
      end when;
      /* Use pre_uSample and pre(c) to break potential algebraic loops by an
       infinitesimal delay if both the continuous and the discrete part
       have direct feedthrough.
    */
      y = pre_uSample + pre(c)*(time - tSample);
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Line(points={{-79.0,-41.0},{-59.0,-33.0},{-40.0,1.0},{-20.0,9.0},{0.0,63.0},{21.0,20.0},{41.0,10.0},{60.0,20.0}},
            color={0,0,127}),
          Line(points={{60.0,20.0},{81.0,10.0}},
            color={0,0,127})}),
        Documentation(info="<html>
<p>
The output signal is the extrapolation through the
values of the last two sampled input signals.
</p>
</html>"));
    end FirstOrderHold;

    block UnitDelay "Unit Delay Block"
      parameter Real y_start=0 "Initial value of output signal";
      extends Interfaces.DiscreteSISO;

    equation
      when sampleTrigger then
        y = pre(u);
      end when;

    initial equation
        y = y_start;
      annotation (
        Documentation(info="<html>
<p>
This block describes a unit delay:
</p>
<blockquote><pre>
     1
y = --- * u
     z
</pre></blockquote>
<p>
that is, the output signal y is the input signal u of the
previous sample instant. Before the second sample instant,
the output y is identical to parameter yStart.
</p>

</html>"),   Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Line(points={{-30.0,0.0},{30.0,0.0}},
          color={0,0,127}),
        Text(textColor={0,0,127},
          extent={{-90.0,10.0},{90.0,90.0}},
          textString="1"),
        Text(textColor={0,0,127},
          extent={{-90.0,-90.0},{90.0,-10.0}},
          textString="z")}));
    end UnitDelay;

    block TransferFunction "Discrete Transfer Function block"
      parameter Real b[:]={1} "Numerator coefficients of transfer function.";
      parameter Real a[:]={1} "Denominator coefficients of transfer function.";
      extends Interfaces.DiscreteSISO;
      output Real x[size(a, 1) - 1](each start=0, each fixed=true)
        "State of transfer function from controller canonical form";
    protected
      parameter Integer nb=size(b, 1) "Size of Numerator of transfer function";
      parameter Integer na=size(a, 1) "Size of Denominator of transfer function";
      Real x1(start=0,fixed=true);
      Real xext[size(a, 1)](each start=0, each fixed=true);

    equation
      when sampleTrigger then
        /* State variables x are defined according to
       controller canonical form. */
        x1 = (u - a[2:size(a, 1)]*pre(x))/a[1];
        xext = vector([x1; pre(x)]);
        x = xext[1:size(x, 1)];
        y = vector([zeros(na - nb, 1); b])*xext;
      end when;
      /* This is a non-sampled equation and above there are two separate
       when-clauses. This breaks feedback loops without direct terms,
       since in that case y will be independent of x1 (and only dependent
       on pre(x)).
    */
      /* Corresponding (simpler) version using when-semantics of Modelica 1.3:
   equation
     when sampleTrigger then
      [x; xn] = [x1; pre(x)];
      [u] = transpose([a])*[x1; pre(x)];
      [y] = transpose([zeros(na - nb, 1); b])*[x1; pre(x)];
     end when;
*/
      annotation (
        Documentation(info="<html>
<p>The <strong>discrete transfer function</strong> block defines the
transfer function between the input signal u and the output
signal y. The numerator has the order nb-1, the denominator
has the order na-1.</p>
<blockquote><pre>
       b(1)*z^(nb-1) + b(2)*z^(nb-2) + ... + b(nb)
y(z) = -------------------------------------------- * u(z)
       a(1)*z^(na-1) + a(2)*z^(na-2) + ... + a(na)
</pre></blockquote>
<p>State variables <strong>x</strong> are defined according to
<strong>controller canonical</strong> form. Initial values of the
states can be set as start values of <strong>x</strong>.</p>
<p>Example:</p>
<blockquote><pre>
Blocks.Discrete.TransferFunction g(b = {2,4}, a = {1,3});
</pre></blockquote>
<p>results in the following transfer function:</p>
<blockquote><pre>
     2*z + 4
y = --------- * u
      z + 3
</pre></blockquote>

</html>",   revisions="<html>
<p><strong>Release Notes:</strong></p>
<ul>
<li><em>November 15, 2000</em>
    by <a href=\"http://www.dynasim.se\">Hans Olsson</a>:<br>
    Converted to when-semantics of Modelica 1.4 with special
    care to avoid unnecessary algebraic loops.</li>
<li><em>June 18, 2000</em>
    by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
    Realized based on a corresponding model of Dieter Moormann
    and Hilding Elmqvist.</li>
</ul>
</html>"),
        Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Line(points={{82.0,0.0},{-84.0,0.0}},
          color={0,0,127}),
        Text(textColor={0,0,127},
          extent={{-92.0,12.0},{86.0,92.0}},
          textString="b(z)"),
        Text(textColor={0,0,127},
          extent={{-90.0,-90.0},{90.0,-12.0}},
          textString="a(z)")}));
    end TransferFunction;

    block StateSpace "Discrete State Space block"
      parameter Real A[:, size(A, 1)]=[1, 0; 0, 1]
        "Matrix A of state space model";
      parameter Real B[size(A, 1), :]=[1; 1] "Matrix B of state space model";
      parameter Real C[:, size(A, 1)]=[1, 1] "Matrix C of state space model";
      parameter Real D[size(C, 1), size(B, 2)]=zeros(size(C, 1), size(B, 2))
        "Matrix D of state space model";

      extends Interfaces.DiscreteMIMO(final nin=size(B, 2), final nout=size(C, 1));
      output Real x[size(A, 1)] "State vector";

    equation
      when sampleTrigger then
        x = A*pre(x) + B*u;
        y = C*pre(x) + D*u;
      end when;
      annotation (
        Documentation(info="<html>
<p>
The <strong>discrete state space</strong> block defines the relation
between the input u and the output y in state space form:
</p>
<blockquote><pre>
x = A * pre(x) + B * u
y = C * pre(x) + D * u
</pre></blockquote>
<p>
where pre(x) is the value of the discrete state x at
the previous sample time instant.
The input is a vector of length nu, the output is a vector
of length ny and nx is the number of states. Accordingly
</p>
<blockquote><pre>
A has the dimension: A(nx,nx),
B has the dimension: B(nx,nu),
C has the dimension: C(ny,nx),
D has the dimension: D(ny,nu)
</pre></blockquote>
<p>
Example:
</p>
<blockquote><pre>
parameter: A = [0.12, 2;3, 1.5]
parameter: B = [2, 7;3, 1]
parameter: C = [0.1, 2]
parameter: D = zeros(ny,nu)

results in the following equations:
  [x[1]]   [0.12  2.00] [pre(x[1])]   [2.0  7.0] [u[1]]
  [    ] = [          ]*[         ] + [        ]*[    ]
  [x[2]]   [3.00  1.50] [pre(x[2])]   [0.1  2.0] [u[2]]
                             [pre(x[1])]            [u[1]]
       y[1]   = [0.1  2.0] * [         ] + [0  0] * [    ]
                             [pre(x[2])]            [u[2]]
</pre></blockquote>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Text(
              extent={{-90,15},{-15,90}},
              textString="A",
              textColor={0,0,127}),
            Text(
              extent={{15,15},{90,90}},
              textString="B",
              textColor={0,0,127}),
            Text(
              extent={{-52,28},{54,-20}},
              textString="z",
              textColor={0,0,127}),
            Text(
              extent={{-90,-15},{-15,-90}},
              textString="C",
              textColor={0,0,127}),
            Text(
              extent={{15,-15},{90,-90}},
              textString="D",
              textColor={0,0,127})}));
    end StateSpace;

    block TriggeredSampler "Triggered sampling of continuous signals"
      extends Blocks.Icons.DiscreteBlock;
      parameter Real y_start=0 "Initial value of output signal";

      Blocks.Interfaces.RealInput u "Connector with a Real input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y "Connector with a Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Blocks.Interfaces.BooleanInput trigger "Trigger input" annotation (
          Placement(transformation(
            origin={0,-118},
            extent={{-20,-20},{20,20}},
            rotation=90)));
    equation
      when trigger then
        y = u;
      end when;
    initial equation
      y = y_start;
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Ellipse(lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{25.0,-10.0},{45.0,10.0}}),
          Line(points={{-100.0,0.0},{-45.0,0.0}},
            color={0,0,127}),
          Line(points={{45.0,0.0},{100.0,0.0}},
            color={0,0,127}),
          Line(points={{0.0,-100.0},{0.0,-26.0}},
            color={255,0,255}),
          Line(points={{-35.0,0.0},{28.0,-48.0}},
            color={0,0,127}),
          Ellipse(lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{-45.0,-10.0},{-25.0,10.0}})}),
        Documentation(info="<html>
<p>
Samples the continuous input signal whenever the trigger input
signal is rising (i.e., trigger changes from <strong>false</strong> to
<strong>true</strong>) and provides the sampled input signal as output.
Before the first sampling, the output signal is equal to
the initial value defined via parameter <strong>y0</strong>.
</p>
</html>"));
    end TriggeredSampler;

    block TriggeredMax
      "Compute maximum, absolute value of continuous signal at trigger instants"

      extends Blocks.Icons.DiscreteBlock;
      Blocks.Interfaces.RealInput u "Connector with a Real input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y "Connector with a Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Blocks.Interfaces.BooleanInput trigger annotation (Placement(
            transformation(
            origin={0,-118},
            extent={{-20,-20},{20,20}},
            rotation=90)));
    equation
      when trigger then
         y = max(pre(y), abs(u));
      end when;
    initial equation
      y = 0;
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Ellipse(lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{25.0,-10.0},{45.0,10.0}}),
          Line(points={{-100.0,0.0},{-45.0,0.0}},
            color={0,0,127}),
          Line(points={{45.0,0.0},{100.0,0.0}},
            color={0,0,127}),
          Line(points={{0.0,-100.0},{0.0,-26.0}},
            color={255,0,255}),
          Line(points={{-35.0,0.0},{28.0,-48.0}},
            color={0,0,127}),
          Text(extent={{-86.0,24.0},{82.0,82.0}},
            color={0,0,127},
            textString="max"),
          Ellipse(lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{-45.0,-10.0},{-25.0,10.0}})}),
        Documentation(info="<html>
<p>
Samples the continuous input signal whenever the trigger input
signal is rising (i.e., trigger changes from <strong>false</strong> to
<strong>true</strong>). The maximum, absolute value of the input signal
at the sampling point is provided as output signal.
</p>
</html>"));
    end TriggeredMax;
    annotation (Documentation(info="<html>
<p>
This package contains <strong>discrete control blocks</strong>
with <strong>fixed sample period</strong>.
Every component of this package is structured in the following way:
</p>
<ol>
<li> A component has <strong>continuous real</strong> input and output signals.</li>
<li> The <strong>input</strong> signals are <strong>sampled</strong> by the given sample period
     defined via parameter <strong>samplePeriod</strong>.
     The first sample instant is defined by parameter <strong>startTime</strong>.</li>
<li> The <strong>output</strong> signals are computed from the sampled input signals.</li>
</ol>
<p>
A <strong>sampled data system</strong> may consist of components of package <strong>Discrete</strong>
and of every other purely <strong>algebraic</strong> input/output block, such
as the components of packages <strong>Modelica.Blocks.Math</strong>,
<strong>Modelica.Blocks.Nonlinear</strong> or <strong>Modelica.Blocks.Sources</strong>.
</p>

</html>",   revisions="<html>
<ul>
<li><em>October 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       New components TriggeredSampler and TriggeredMax added.</li>
<li><em>June 18, 2000</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized based on a corresponding library of Dieter Moormann and
       Hilding Elmqvist.</li>
</ul>
</html>"),   Icon(graphics={
          Line(points={{-88,0},{-45,0}}, color={95,95,95}),
          Ellipse(
            lineColor={95,95,95},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{-45,-10},{-25,10}}),
          Line(points={{-35,0},{24,52}}, color={95,95,95}),
          Ellipse(
            lineColor={95,95,95},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{25,-10},{45,10}}),
          Line(points={{45,0},{82,0}}, color={95,95,95})}));
  end Discrete;

  package Interaction
    "Library of user interaction blocks to input and to show variables in a diagram animation"
    extends Modelica.Icons.Package;

    package Show "Library of blocks to show variables in a diagram animation"
      extends Modelica.Icons.Package;

      block RealValue
        "Show Real value from numberPort or from number input field in diagram layer dynamically"
        parameter Boolean use_numberPort = true "= true, if numberPort enabled"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        input Real number = 0.0
          "Number to visualize if use_numberPort=false (time varying)"
          annotation(Dialog(enable=not use_numberPort), HideResult=true);
        parameter Integer significantDigits(min=1) = 2
          "Number of significant digits to be shown";

        Blocks.Interfaces.RealInput numberPort if use_numberPort
          "Number to be shown in diagram layer if use_numberPort = true"
          annotation (HideResult=true, Placement(transformation(extent={{-130,-15},
                  {-100,15}})));
         Blocks.Interfaces.RealOutput showNumber;
      equation
        if use_numberPort then
           connect(numberPort, showNumber);
        else
           showNumber = number;
        end if;

        annotation (Icon(
            coordinateSystem(preserveAspectRatio=false,
              extent={{-100.0,-100.0},{100.0,100.0}}),
              graphics={
            Rectangle(lineColor={0,0,127},
              fillColor={236,233,216},
              fillPattern=FillPattern.Solid,
              lineThickness=5.0,
              borderPattern=BorderPattern.Raised,
              extent={{-100.0,-40.0},{100.0,40.0}}),
            Text(extent={{-94.0,-34.0},{96.0,34.0}},
              textString=DynamicSelect("0.0", String(showNumber, significantDigits=significantDigits))),
            Text(visible=not use_numberPort,
              extent={{-150.0,-70.0},{150.0,-50.0}},
              textString="%number")}), Documentation(info="<html>
<p>
This block visualizes a Real number in a diagram animation.
The number to be visualized can be defined in the following ways:
</p>

<ul>
<li> If useNumberPort = <strong>true</strong> (which is the default), a Real
     input is present and this input variable is shown.</li>

<li> If useNumberPort = <strong>false</strong> no input connector is present.
     Instead, a Real input field is activated in the parameter menu
     and the Real expression from this input menu is shown.</li>
</ul>

<p>
The two versions of the block are shown in the following image (in the right variant, the
name of the variable value that is displayed is also shown below the icon):
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Interaction/Show/RealValue.png\"
     alt=\"RealValue.png\">
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.RealNetwork1\">Modelica.Blocks.Examples.RealNetwork1</a>.
</p>
</html>"));
      end RealValue;

      block IntegerValue
        "Show Integer value from numberPort or from number input field in diagram layer dynamically"
        parameter Boolean use_numberPort = true "= true, if numberPort enabled"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        input Integer number=0
          "Number to visualize if use_numberPort=false (time varying)"
          annotation(Dialog(enable=not use_numberPort), HideResult=true);
        Blocks.Interfaces.IntegerInput numberPort if use_numberPort
          "Number to be shown in diagram layer if use_numberPort = true"
          annotation (HideResult=true, Placement(transformation(extent={{-130,-15},
                  {-100,15}})));
         Blocks.Interfaces.IntegerOutput showNumber;
      equation
        if use_numberPort then
           connect(numberPort, showNumber);
        else
           showNumber = number;
        end if;

        annotation (Icon(
            coordinateSystem(preserveAspectRatio=false,
              extent={{-100.0,-100.0},{100.0,100.0}}),
              graphics={
            Rectangle(lineColor={0,0,127},
              fillColor={236,233,216},
              fillPattern=FillPattern.Solid,
              lineThickness=5.0,
              borderPattern=BorderPattern.Raised,
              extent={{-100.0,-40.0},{100.0,40.0}}),
            Text(extent={{-94.0,-34.0},{96.0,34.0}},
              textString=DynamicSelect("0", String(showNumber))),
            Text(visible=not use_numberPort,
              extent={{-150.0,-70.0},{150.0,-50.0}},
              textString="%number")}), Documentation(info="<html>
<p>
This block visualizes an Integer number in a diagram animation.
The number to be visualized can be defined in the following ways:
</p>

<ul>
<li> If useNumberPort = <strong>true</strong> (which is the default), an Integer
     input is present and this input variable is shown.</li>

<li> If useNumberPort = <strong>false</strong> no input connector is present.
     Instead, an Integer input field is activated in the parameter menu
     and the Integer expression from this input menu is shown.</li>
</ul>

<p>
The two versions of the block are shown in the following image (in the right variant, the
name of the variable value that is displayed is also shown below the icon):
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Interaction/Show/IntegerValue.png\"
     alt=\"IntegerValue.png\">
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.IntegerNetwork1\">Modelica.Blocks.Examples.IntegerNetwork1</a>.
</p>
</html>"));
      end IntegerValue;

      block BooleanValue
        "Show Boolean value from numberPort or from number input field in diagram layer dynamically"
        parameter Boolean use_activePort = true "= true, if activePort enabled"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        input Boolean active=false
          "Boolean variable to visualize if use_activePort=false (time varying)"
          annotation(Dialog(enable=not use_activePort),HideResult=true);
        Blocks.Interfaces.BooleanInput activePort if use_activePort
          "Boolean variable to be shown in diagram layer if use_activePort = true"
          annotation (HideResult=true, Placement(transformation(extent={{-130,-15},
                  {-100,15}})));

         Blocks.Interfaces.BooleanOutput showActive;
      equation
        if use_activePort then
           connect(activePort, showActive);
        else
           showActive = active;
        end if;

        annotation (Icon(
            coordinateSystem(preserveAspectRatio=false,
              extent={{-100.0,-100.0},{100.0,100.0}}),
              graphics={
            Text(visible=not use_activePort,
              extent={{-188.0,-80.0},{62.0,-60.0}},
              textString="%active"),
            Ellipse(lineColor={64,64,64},
              fillColor=DynamicSelect({192,192,192}, if showActive then {0,255,0} else {235,235,235}),
              pattern=LinePattern.None,
              fillPattern=FillPattern.Sphere,
              extent={{-100.0,-40.0},{-20.0,40.0}})}), Documentation(info="<html>
<p>
This block visualizes a Boolean variable in a diagram animation.
The Boolean variable to be visualized can be defined in the following ways:
</p>

<ul>
<li> If useActivePort = <strong>true</strong> (which is the default), a Boolean
     input is present and this input variable is shown.</li>

<li> If useActivePort = <strong>false</strong> no input connector is present.
     Instead, a Boolean input field is activated in the parameter menu
     and the Boolean expression from this input menu is shown.</li>
</ul>

<p>
If the Boolean variable is <strong>false</strong> the block is \"grey\", otherwise, it is \"green\".
The two versions of the block are shown in the following image (in the right variant, the
name of the variable value that is displayed is also shown below the icon):
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Interaction/Show/BooleanValue.png\"
     alt=\"BooleanValue.png\">
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>
</html>"));
      end BooleanValue;
    end Show;
    annotation (Icon(graphics={Text(
            extent={{-98,-30},{96,34}},
            textString="0")}));
  end Interaction;

  package Interfaces
    "Library of connectors and partial models for input/output blocks"

    extends Modelica.Icons.InterfacesPackage;

    connector RealInput = input Real "'input Real' as connector" annotation (
      defaultComponentName="u",
      Icon(graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})},
        coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          initialScale=0.2,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid,
          points={{0.0,50.0},{100.0,0.0},{0.0,-50.0},{0.0,50.0}}),
        Text(
          textColor={0,0,127},
          extent={{-10.0,60.0},{-10.0,85.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one input signal of type Real.
</p>
</html>"));

    connector RealOutput = output Real "'output Real' as connector" annotation (
      defaultComponentName="y",
      Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,100.0},{100.0,0.0},{-100.0,-100.0}})}),
      Diagram(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          points={{-100.0,50.0},{0.0,0.0},{-100.0,-50.0}}),
        Text(
          textColor={0,0,127},
          extent={{30.0,60.0},{30.0,110.0}},
          textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one output signal of type Real.
</p>
</html>"));

    connector BooleanInput = input Boolean "'input Boolean' as connector"
      annotation (
      defaultComponentName="u",
      Icon(graphics={Polygon(
            points={{-100,100},{100,0},{-100,-100},{-100,100}},
            lineColor={255,0,255},
            fillColor={255,0,255},
            fillPattern=FillPattern.Solid)}, coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          initialScale=0.2,
          extent={{-100,-100},{100,100}}), graphics={Polygon(
            points={{0,50},{100,0},{0,-50},{0,50}},
            lineColor={255,0,255},
            fillColor={255,0,255},
            fillPattern=FillPattern.Solid), Text(
            extent={{-10,85},{-10,60}},
            textColor={255,0,255},
            textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one input signal of type Boolean.
</p>
</html>"));

    connector BooleanOutput = output Boolean "'output Boolean' as connector"
      annotation (
      defaultComponentName="y",
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Polygon(
            points={{-100,100},{100,0},{-100,-100},{-100,100}},
            lineColor={255,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Polygon(
            points={{-100,50},{0,0},{-100,-50},{-100,50}},
            lineColor={255,0,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid), Text(
            extent={{30,110},{30,60}},
            textColor={255,0,255},
            textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one output signal of type Boolean.
</p>
</html>"));

    connector IntegerInput = input Integer "'input Integer' as connector"
      annotation (
      defaultComponentName="u",
      Icon(graphics={Polygon(
            points={{-100,100},{100,0},{-100,-100},{-100,100}},
            lineColor={255,127,0},
            fillColor={255,127,0},
            fillPattern=FillPattern.Solid)}, coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          initialScale=0.2,
          extent={{-100,-100},{100,100}}), graphics={Polygon(
            points={{0,50},{100,0},{0,-50},{0,50}},
            lineColor={255,127,0},
            fillColor={255,127,0},
            fillPattern=FillPattern.Solid), Text(
            extent={{-10,85},{-10,60}},
            textColor={255,127,0},
            textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one input signal of type Integer.
</p>
</html>"));

    connector IntegerOutput = output Integer "'output Integer' as connector"
      annotation (
      defaultComponentName="y",
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Polygon(
            points={{-100,100},{100,0},{-100,-100},{-100,100}},
            lineColor={255,127,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}),
      Diagram(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{100,100}}), graphics={Polygon(
            points={{-100,50},{0,0},{-100,-50},{-100,50}},
            lineColor={255,127,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid), Text(
            extent={{30,110},{30,60}},
            textColor={255,127,0},
            textString="%name")}),
      Documentation(info="<html>
<p>
Connector with one output signal of type Integer.
</p>
</html>"));

    connector RealVectorInput = input Real
      "Real input connector used for vector of connectors" annotation (
      defaultComponentName="u",
      Icon(graphics={Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,127},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid)}, coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          initialScale=0.2,
          extent={{-100,-100},{100,100}}), graphics={Text(
            extent={{-10,85},{-10,60}},
            textColor={0,0,127},
            textString="%name"), Ellipse(
            extent={{-50,50},{50,-50}},
            lineColor={0,0,127},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>
<p>
Real input connector that is used for a vector of connectors,
for example <a href=\"modelica://Modelica.Blocks.Interfaces.PartialRealMISO\">PartialRealMISO</a>,
and has therefore a different icon as RealInput connector.
</p>
</html>"));

    connector IntegerVectorInput = input Integer
      "Integer input connector used for vector of connectors" annotation (
      defaultComponentName="u",
      Icon(graphics={Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={255,128,0},
            fillColor={255,128,0},
            fillPattern=FillPattern.Solid)}, coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          initialScale=0.2,
          extent={{-100,-100},{100,100}}), graphics={Text(
            extent={{-10,85},{-10,60}},
            textColor={255,128,0},
            textString="%name"), Ellipse(
            extent={{-50,50},{50,-50}},
            lineColor={255,128,0},
            fillColor={255,128,0},
            fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>

<p>
Integer input connector that is used for a vector of connectors,
for example <a href=\"modelica://Modelica.Blocks.Interfaces.PartialIntegerMISO\">PartialIntegerMISO</a>,
and has therefore a different icon as IntegerInput connector.
</p>
</html>"));

    connector BooleanVectorInput = input Boolean
      "Boolean input connector used for vector of connectors" annotation (
      defaultComponentName="u",
      Icon(graphics={Ellipse(
            extent={{-100,-100},{100,100}},
            lineColor={255,0,255},
            fillColor={255,0,255},
            fillPattern=FillPattern.Solid)}, coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=false,
          initialScale=0.2)),
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          initialScale=0.2,
          extent={{-100,-100},{100,100}}), graphics={Text(
            extent={{-10,85},{-10,60}},
            textColor={255,0,255},
            textString="%name"), Ellipse(
            extent={{-50,50},{50,-50}},
            lineColor={255,0,255},
            fillColor={255,0,255},
            fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>
<p>
Boolean input connector that is used for a vector of connectors,
for example <a href=\"modelica://Modelica.Blocks.Interfaces.PartialBooleanMISO\">PartialBooleanMISO</a>,
and has therefore a different icon as BooleanInput connector.
</p>
</html>"));

    connector RealVectorOutput = output Real
      "Real output connector used for vector of connectors" annotation (
      defaultComponentName="y",
      Icon(graphics={Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,127},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid)}, coordinateSystem(
          extent={{-100,-100},{100,100}},
          preserveAspectRatio=true,
          initialScale=0.2)),
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          initialScale=0.2,
          extent={{-100,-100},{100,100}}), graphics={Text(
            extent={{-10,85},{-10,60}},
            textColor={0,0,127},
            textString="%name"), Ellipse(
            extent={{-50,50},{50,-50}},
            lineColor={0,0,127},
            fillColor={0,0,127},
            fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>
<p>
Real output connector that is used for a vector of connectors,
for example <a href=\"modelica://Modelica.Blocks.Routing.DeMultiplex\">DeMultiplex</a>,
and has therefore a different icon as RealOutput connector.
</p>
</html>"));

    partial block SO "Single Output continuous control block"
      extends Blocks.Icons.Block;

      RealOutput y "Connector of Real output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has one continuous Real output signal.
</p>
</html>"));

    end SO;

    partial block MO "Multiple Output continuous control block"
      extends Blocks.Icons.Block;

      parameter Integer nout(min=1) = 1 "Number of outputs";
      RealOutput y[nout] "Connector of Real output signals" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has one continuous Real output signal vector.
</p>
</html>"));

    end MO;

    partial block SISO "Single Input Single Output continuous control block"
      extends Blocks.Icons.Block;

      RealInput u "Connector of Real input signal" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      RealOutput y "Connector of Real output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has one continuous Real input and one continuous Real output signal.
</p>
</html>"));
    end SISO;

    partial block SI2SO
      "2 Single Input / 1 Single Output continuous control block"
      extends Blocks.Icons.Block;

      RealInput u1 "Connector of Real input signal 1" annotation (Placement(
            transformation(extent={{-140,40},{-100,80}})));
      RealInput u2 "Connector of Real input signal 2" annotation (Placement(
            transformation(extent={{-140,-80},{-100,-40}})));
      RealOutput y "Connector of Real output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));

      annotation (Documentation(info="<html>
<p>
Block has two continuous Real input signals u1 and u2 and one
continuous Real output signal y.
</p>
</html>"));

    end SI2SO;

    partial block SIMO "Single Input Multiple Output continuous control block"
      extends Blocks.Icons.Block;
      parameter Integer nout=1 "Number of outputs";
      RealInput u "Connector of Real input signal" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      RealOutput y[nout] "Connector of Real output signals" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));

      annotation (Documentation(info="<html>
<p> Block has one continuous Real input signal and a
    vector of continuous Real output signals.</p>

</html>"));
    end SIMO;

    partial block MISO "Multiple Input Single Output continuous control block"

      extends Blocks.Icons.Block;
      parameter Integer nin=1 "Number of inputs";
      RealInput u[nin] "Connector of Real input signals" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      RealOutput y "Connector of Real output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has a vector of continuous Real input signals and
one continuous Real output signal.
</p>
</html>"));
    end MISO;

    partial block PartialRealMISO
      "Partial block with a RealVectorInput and a RealOutput signal"

      parameter Integer significantDigits(min=1) = 3
        "Number of significant digits to be shown in dynamic diagram layer for y"
        annotation (Dialog(tab="Advanced"));
      parameter Integer nu(min=0) = 0 "Number of input connections"
        annotation (Dialog(connectorSizing=true), HideResult=true);
      Blocks.Interfaces.RealVectorInput u[nu]
        annotation (Placement(transformation(extent={{-120,70},{-80,-70}})));
      Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{100,-17},{134,17}})));
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            initialScale=0.06), graphics={
            Text(
              extent={{110,-50},{300,-70}},
              textString=DynamicSelect(" ", String(y, significantDigits=
                  significantDigits))),
            Text(
              extent={{-250,170},{250,110}},
              textString="%name",
              textColor={0,0,255}),
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={255,137,0},
              fillColor={255,255,255},
              borderPattern=BorderPattern.Raised,
              fillPattern=FillPattern.Solid)}));
    end PartialRealMISO;

    partial block MIMO "Multiple Input Multiple Output continuous control block"

      extends Blocks.Icons.Block;
      parameter Integer nin=1 "Number of inputs";
      parameter Integer nout=1 "Number of outputs";
      RealInput u[nin] "Connector of Real input signals" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      RealOutput y[nout] "Connector of Real output signals" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has a continuous Real input and a continuous Real output signal vector.
The signal sizes of the input and output vector may be different.
</p>
</html>"));
    end MIMO;

    partial block MIMOs
      "Multiple Input Multiple Output continuous control block with same number of inputs and outputs"

      extends Blocks.Icons.Block;
      parameter Integer n=1 "Number of inputs (= number of outputs)";
      RealInput u[n] "Connector of Real input signals" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      RealOutput y[n] "Connector of Real output signals" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has a continuous Real input and a continuous Real output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"));
    end MIMOs;

    partial block MI2MO
      "2 Multiple Input / Multiple Output continuous control block"
      extends Blocks.Icons.Block;

      parameter Integer n=1 "Dimension of input and output vectors.";

      RealInput u1[n] "Connector 1 of Real input signals" annotation (Placement(
            transformation(extent={{-140,40},{-100,80}})));
      RealInput u2[n] "Connector 2 of Real input signals" annotation (Placement(
            transformation(extent={{-140,-80},{-100,-40}})));
      RealOutput y[n] "Connector of Real output signals" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has two continuous Real input vectors u1 and u2 and one
continuous Real output vector y.
All vectors have the same number of elements.
</p>
</html>"));

    end MI2MO;

    partial block SignalSource "Base class for continuous signal source"
      extends SO;
      parameter Real offset=0 "Offset of output signal y";
      parameter SI.Time startTime=0 "Output y = offset for time < startTime";
      annotation (Documentation(info="<html>
<p>
Basic block for Real sources of package Blocks.Sources.
This component has one continuous Real output signal y
and two parameters (offset, startTime) to shift the
generated signal.
</p>
</html>"));
    end SignalSource;

    partial block SVcontrol "Single-Variable continuous controller"
      extends Blocks.Icons.Block;

      RealInput u_s "Connector of setpoint input signal" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      RealInput u_m "Connector of measurement input signal" annotation (Placement(
            transformation(
            origin={0,-120},
            extent={{20,-20},{-20,20}},
            rotation=270)));
      RealOutput y "Connector of actuator output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has two continuous Real input signals and one
continuous Real output signal. The block is designed
to be used as base class for a corresponding controller.
</p>
</html>"));
    end SVcontrol;

    partial block MVcontrol "Multi-Variable continuous controller"
      extends Blocks.Icons.Block;

      parameter Integer nu_s=1 "Number of setpoint inputs";
      parameter Integer nu_m=1 "Number of measurement inputs";
      parameter Integer ny=1 "Number of actuator outputs";
      RealInput u_s[nu_s] "Connector of setpoint input signals" annotation (
          Placement(transformation(extent={{-140,-20},{-100,20}})));
      RealInput u_m[nu_m] "Connector of measurement input signals" annotation (
          Placement(transformation(
            origin={0,-120},
            extent={{20,-20},{-20,20}},
            rotation=270)));
      RealOutput y[ny] "Connector of actuator output signals" annotation (
          Placement(transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has two continuous Real input signal vectors and one
continuous Real output signal vector. The block is designed
to be used as base class for a corresponding controller.
</p>
</html>"));
    end MVcontrol;

    partial block DiscreteBlock "Base class of discrete control blocks"
      extends Blocks.Icons.DiscreteBlock;

      parameter SI.Time samplePeriod(min=100*Modelica.Constants.eps, start=0.1)
        "Sample period of component";
      parameter SI.Time startTime=0 "First sample time instant";
    protected
      output Boolean sampleTrigger "True, if sample time instant";
      output Boolean firstTrigger(start=false, fixed=true)
        "Rising edge signals first sample instant";
    equation
      sampleTrigger = sample(startTime, samplePeriod);
      when sampleTrigger then
        firstTrigger = time <= startTime + samplePeriod/2;
      end when;
      annotation (Documentation(info="<html>
<p>
Basic definitions of a discrete block of library
Blocks.Discrete.
The output(s) will only change at events, but are not formally a discrete variable(s) in Modelica.
The input(s) will be sampled, and can thus be continuous variable(s).
</p>

<p>
<strong>Important</strong>: If you connect several discrete blocks you should normally ensure that <strong>samplePeriod (and startTime)
are exactly identical</strong> for all blocks, since otherwise the output from one block will be transformed into a continuous signal
and sampled, which can cause a variable delay of up to one sample period leading to unexpected results.
</p>

<p>
Modelica 3.3 introduced synchronous operators that avoid the need to manually propagate samplePeriod to each block.
</p>
</html>"));
    end DiscreteBlock;

    partial block DiscreteSISO
      "Single Input Single Output discrete control block"

      extends DiscreteBlock;

      Blocks.Interfaces.RealInput u "Connector of Real input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has one input and one output signal
which are sampled due to the defined <strong>samplePeriod</strong> parameter.
See the base-class <a href=\"modelica://Modelica.Blocks.Interfaces.DiscreteBlock\">DiscreteBlock</a> for more information.
</p>
</html>"));
    end DiscreteSISO;

    partial block DiscreteMIMO
      "Multiple Input Multiple Output discrete control block"

      extends DiscreteBlock;
      parameter Integer nin=1 "Number of inputs";
      parameter Integer nout=1 "Number of outputs";

      Blocks.Interfaces.RealInput u[nin] "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y[nout] "Connector of Real output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      annotation (Documentation(info="<html>
<p>
Block has a input and a output signal vector
which are sampled due to the defined <strong>samplePeriod</strong> parameter.
See the base-class <a href=\"modelica://Modelica.Blocks.Interfaces.DiscreteBlock\">DiscreteBlock</a> for more information.
</p>
</html>"));
    end DiscreteMIMO;

    partial block DiscreteMIMOs
      "Multiple Input Multiple Output discrete control block"
      parameter Integer n=1 "Number of inputs (= number of outputs)";
      extends DiscreteBlock;

      Blocks.Interfaces.RealInput u[n] "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y[n] "Connector of Real output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      annotation (Documentation(info="<html>
<p>
Block has a input and a output signal vector
where the signal sizes of the input and output vector are identical.
These signals are sampled due to the defined <strong>samplePeriod</strong> parameter.
See the base-class <a href=\"modelica://Modelica.Blocks.Interfaces.DiscreteBlock\">DiscreteBlock</a> for more information.
</p>
</html>"));

    end DiscreteMIMOs;

    partial block SVdiscrete "Discrete Single-Variable controller"
      extends DiscreteBlock;

      Discrete.Sampler sampler_s(final samplePeriod=samplePeriod, final startTime=
           startTime) annotation (Placement(transformation(extent={{-100,-10},{-80,
                10}})));
      Discrete.Sampler sampler_m(final samplePeriod=samplePeriod, final startTime=
           startTime) annotation (Placement(transformation(
            origin={0,-90},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Blocks.Interfaces.RealInput u_s "Scalar setpoint input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealInput u_m "Scalar measurement input signal"
        annotation (Placement(transformation(
            origin={0,-120},
            extent={{20,-20},{-20,20}},
            rotation=270)));
      Blocks.Interfaces.RealOutput y "Scalar actuator output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(u_s, sampler_s.u) annotation (Line(points={{-120,0},{-102,0}}));
      connect(u_m, sampler_m.u)
        annotation (Line(points={{0,-120},{0,-111},{0,-102}}));
      annotation (Documentation(info="<html>
<p>
Block has two Real input signals and one
Real output signal
that are sampled due to the defined <strong>samplePeriod</strong> parameter.
The block is designed
to be used as base class for a corresponding controller.
See the base-class <a href=\"modelica://Modelica.Blocks.Interfaces.DiscreteBlock\">DiscreteBlock</a> for more information.
</p>
</html>"));
    end SVdiscrete;

    partial block MVdiscrete "Discrete Multi-Variable controller"
      extends DiscreteBlock;
      parameter Integer nu_s=1 "Number of setpoint inputs";
      parameter Integer nu_m=1 "Number of measurement inputs";
      parameter Integer ny=1 "Number of actuator outputs";
      Discrete.Sampler sampler_s[nu_s](each final samplePeriod=samplePeriod,
          each final startTime=startTime) annotation (Placement(transformation(
              extent={{-90,-10},{-70,10}})));
      Discrete.Sampler sampler_m[nu_m](each final samplePeriod=samplePeriod,
          each final startTime=startTime) annotation (Placement(transformation(
            origin={0,-80},
            extent={{-10,-10},{10,10}},
            rotation=90)));
      Blocks.Interfaces.RealInput u_s[nu_s] "Setpoint input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealInput u_m[nu_m] "Measurement input signals"
        annotation (Placement(transformation(
            origin={0,-120},
            extent={{20,-20},{-20,20}},
            rotation=270)));
      Blocks.Interfaces.RealOutput y[ny] "Actuator output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      connect(u_s, sampler_s.u) annotation (Line(points={{-120,0},{-92,0}}));
      connect(u_m, sampler_m.u)
        annotation (Line(points={{0,-120},{0,-106},{0,-92}}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={Text(
                extent={{-100,-10},{-80,-30}},
                textString="u_s",
                textColor={0,0,255})}),         Documentation(info="<html>
<p>
Block has two Real input signal vectors and one
Real output signal vector. The vector signals
are sampled due to the defined <strong>samplePeriod</strong> parameter.
The block is designed
to be used as base class for a corresponding controller.
See the base-class <a href=\"modelica://Modelica.Blocks.Interfaces.DiscreteBlock\">DiscreteBlock</a> for more information.
</p>
</html>"));
    end MVdiscrete;

    partial block BooleanSISO
      "Single Input Single Output control block with signals of type Boolean"

      extends Blocks.Icons.BooleanBlock;

    public
      BooleanInput u "Connector of Boolean input signal" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      BooleanOutput y "Connector of Boolean output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));

      annotation (Documentation(info="<html>
<p>
Block has one continuous Boolean input and one continuous Boolean output signal.
</p>
</html>"));
    end BooleanSISO;

    partial block BooleanMIMOs
      "Multiple Input Multiple Output continuous control block with same number of inputs and outputs of Boolean type"

      extends Blocks.Icons.BooleanBlock;
      parameter Integer n=1 "Number of inputs (= number of outputs)";
      BooleanInput u[n] "Connector of Boolean input signals" annotation (
          Placement(transformation(extent={{-140,-20},{-100,20}})));
      BooleanOutput y[n] "Connector of Boolean output signals" annotation (
          Placement(transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has a continuous Boolean input and a continuous Boolean output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"));
    end BooleanMIMOs;

    partial block MI2BooleanMOs
      "2 Multiple Input / Boolean Multiple Output block with same signal lengths"

      extends Blocks.Icons.BooleanBlock;
      parameter Integer n=1 "Dimension of input and output vectors.";
      RealInput u1[n] "Connector 1 of Boolean input signals" annotation (
          Placement(transformation(extent={{-140,40},{-100,80}})));
      RealInput u2[n] "Connector 2 of Boolean input signals" annotation (
          Placement(transformation(extent={{-140,-80},{-100,-40}})));
      BooleanOutput y[n] "Connector of Boolean output signals" annotation (
          Placement(transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>Block has two Boolean input vectors u1 and u2 and one Boolean output
vector y. All vectors have the same number of elements.</p>
</html>"));
    end MI2BooleanMOs;

    partial block SI2BooleanSO "2 Single Input / Boolean Single Output block"

      extends Blocks.Icons.BooleanBlock;
      BooleanInput u1 "Connector 1 of Boolean input signals" annotation (
          Placement(transformation(extent={{-140,40},{-100,80}})));
      BooleanInput u2 "Connector 2 of Boolean input signals" annotation (
          Placement(transformation(extent={{-140,-80},{-100,-40}})));
      BooleanOutput y "Connector of Boolean output signals" annotation (
          Placement(transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has two Boolean input signals u1 and u2 and one Boolean output signal y.
</p>
</html>"));

    end SI2BooleanSO;

    partial block BooleanSignalSource "Base class for Boolean signal sources"

      extends Blocks.Icons.BooleanBlock;
      BooleanOutput y "Connector of Boolean output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{68,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
Basic block for Boolean sources of package Blocks.Sources.
This component has one continuous Boolean output signal y.
</p>
</html>"));

    end BooleanSignalSource;

    partial block IntegerSO "Single Integer Output continuous control block"
      extends Blocks.Icons.IntegerBlock;

      IntegerOutput y "Connector of Integer output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has one continuous Integer output signal.
</p>
</html>"));
    end IntegerSO;

    partial block IntegerMO "Multiple Integer Output continuous control block"
      extends Blocks.Icons.IntegerBlock;

      parameter Integer nout(min=1) = 1 "Number of outputs";
      IntegerOutput y[nout] "Connector of Integer output signals" annotation (
          Placement(transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has one continuous Integer output signal vector.
</p>
</html>"));
    end IntegerMO;

    partial block IntegerSignalSource
      "Base class for continuous Integer signal source"
      extends IntegerSO;
      parameter Integer offset=0 "Offset of output signal y";
      parameter SI.Time startTime=0 "Output y = offset for time < startTime";
      annotation (Documentation(info="<html>
<p>
Basic block for Integer sources of package Blocks.Sources.
This component has one continuous Integer output signal y
and two parameters (offset, startTime) to shift the
generated signal.
</p>
</html>"));
    end IntegerSignalSource;

    partial block IntegerSIBooleanSO
      "Integer Input Boolean Output continuous control block"

      extends Blocks.Icons.BooleanBlock;
      IntegerInput u "Connector of Integer input signal" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      BooleanOutput y "Connector of Boolean output signal" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has a continuous Integer input and a continuous Boolean output signal.
</p>
</html>"));
    end IntegerSIBooleanSO;

    partial block IntegerMIBooleanMOs
      "Multiple Integer Input Multiple Boolean Output continuous control block with same number of inputs and outputs"

      extends Blocks.Icons.BooleanBlock;
      parameter Integer n=1 "Number of inputs (= number of outputs)";
      IntegerInput u[n] "Connector of Integer input signals" annotation (
          Placement(transformation(extent={{-140,-20},{-100,20}})));
      BooleanOutput y[n] "Connector of Boolean output signals" annotation (
          Placement(transformation(extent={{100,-10},{120,10}})));
      annotation (Documentation(info="<html>
<p>
Block has a continuous Integer input and a continuous Boolean output signal vector
where the signal sizes of the input and output vector are identical.
</p>
</html>"));
    end IntegerMIBooleanMOs;

    partial block PartialIntegerSISO
      "Partial block with a IntegerInput and an IntegerOutput signal"

      Blocks.Interfaces.IntegerInput u "Integer input signal"
        annotation (Placement(transformation(extent={{-180,-40},{-100,40}})));
      Blocks.Interfaces.IntegerOutput y "Integer output signal"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            initialScale=0.06), graphics={
            Text(
              extent={{110,-50},{250,-70}},
              textString=DynamicSelect(" ", String(
                    y,
                    minimumLength=1,
                    significantDigits=0))),
            Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255}),
            Rectangle(
              extent={{-100,100},{100,-100}},
              fillColor={255,213,170},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised)}));
    end PartialIntegerSISO;

    partial block PartialIntegerMISO
      "Partial block with an IntegerVectorInput and an IntegerOutput signal"

      parameter Integer nu(min=0) = 0 "Number of input connections"
        annotation (Dialog(connectorSizing=true), HideResult=true);
      Blocks.Interfaces.IntegerVectorInput u[nu]
        "Vector of Integer input signals"
        annotation (Placement(transformation(extent={{-120,70},{-80,-70}})));
      Blocks.Interfaces.IntegerOutput y "Integer output signal"
        annotation (Placement(transformation(extent={{100,-15},{130,15}})));
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            initialScale=0.06), graphics={
            Text(
              extent={{110,-50},{250,-70}},
              textString=DynamicSelect(" ", String(
                    y,
                    minimumLength=1,
                    significantDigits=0))),
            Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255}),
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={255,137,0},
              fillColor={255,213,170},
              borderPattern=BorderPattern.Raised,
              fillPattern=FillPattern.Solid)}));
    end PartialIntegerMISO;

    partial block partialBooleanSISO
      "Partial block with 1 input and 1 output Boolean signal"
      extends Blocks.Icons.PartialBooleanBlock;
      Blocks.Interfaces.BooleanInput u "Connector of Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Ellipse(
              extent={{-71,7},{-85,-7}},
              lineColor=DynamicSelect({235,235,235}, if u then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if u then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid), Ellipse(
              extent={{71,7},{85,-7}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
Block has one continuous Boolean input and one continuous Boolean output signal
with a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

    end partialBooleanSISO;

    partial block partialBooleanSI2SO
      "Partial block with 2 input and 1 output Boolean signal"
      extends Blocks.Icons.PartialBooleanBlock;
      Blocks.Interfaces.BooleanInput u1
        "Connector of first Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.BooleanInput u2
        "Connector of second Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
      Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-71,7},{-85,-7}},
              lineColor=DynamicSelect({235,235,235}, if u1 then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if u1 then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-71,-74},{-85,-88}},
              lineColor=DynamicSelect({235,235,235}, if u2 then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if u2 then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{71,7},{85,-7}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
Block has two continuous Boolean input and one continuous Boolean output signal
with a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

    end partialBooleanSI2SO;

    partial block partialBooleanSI3SO
      "Partial block with 3 input and 1 output Boolean signal"
      extends Blocks.Icons.PartialBooleanBlock;
      Blocks.Interfaces.BooleanInput u1
        "Connector of first Boolean input signal"
        annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
      Blocks.Interfaces.BooleanInput u2
        "Connector of second Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.BooleanInput u3
        "Connector of third Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
      Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-71,74},{-85,88}},
              lineColor=DynamicSelect({235,235,235}, if u1 then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if u1 then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-71,7},{-85,-7}},
              lineColor=DynamicSelect({235,235,235}, if u2 then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if u2 then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-71,-74},{-85,-88}},
              lineColor=DynamicSelect({235,235,235}, if u3 then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if u3 then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{71,7},{85,-7}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid)}), Documentation(info="<html><p>
Block has three continuous Boolean input and one continuous Boolean output signal
with a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

    end partialBooleanSI3SO;

    partial block partialBooleanSI "Partial block with 1 input Boolean signal"
      extends Blocks.Icons.PartialBooleanBlock;

      Blocks.Interfaces.BooleanInput u "Connector of Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Ellipse(
              extent={{-71,7},{-85,-7}},
              lineColor=DynamicSelect({235,235,235}, if u then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if u then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
Block has one continuous Boolean input signal
with a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

    end partialBooleanSI;

    partial block partialBooleanSO "Partial block with 1 output Boolean signal"

      Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      extends Blocks.Icons.PartialBooleanBlock;

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Ellipse(
              extent={{71,7},{85,-7}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
Block has one continuous Boolean output signal
with a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

    end partialBooleanSO;

    partial block partialBooleanSource
      "Partial source block (has 1 output Boolean signal and an appropriate default icon)"
      extends Blocks.Icons.PartialBooleanBlock;

      Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{-80,88},{-88,66},{-72,66},{-80,88}},
              lineColor={255,0,255},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,66},{-80,-82}}, color={255,0,255}),
            Line(points={{-90,-70},{72,-70}}, color={255,0,255}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={255,0,255},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{71,7},{85,-7}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
Basic block for Boolean sources of package Blocks.Sources.
This component has one continuous Boolean output signal y
and a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

    end partialBooleanSource;

    partial block partialBooleanThresholdComparison
      "Partial block to compare the Real input u with a threshold and provide the result as 1 Boolean output signal"

      parameter Real threshold=0 "Comparison with respect to threshold";

      Blocks.Interfaces.RealInput u "Connector of Real input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              fillColor={210,210,210},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised),
            Text(
              extent={{-150,-140},{150,-110}},
              textString="%threshold"),
            Ellipse(
              extent={{71,7},{85,-7}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid), Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255})}), Documentation(info="<html>
<p>
Block has one continuous Real input and one continuous Boolean output signal
as well as a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

    end partialBooleanThresholdComparison;

    partial block partialBooleanComparison
      "Partial block with 2 Real input and 1 Boolean output signal (the result of a comparison of the two Real inputs)"

      Blocks.Interfaces.RealInput u1 "Connector of first Real input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealInput u2 "Connector of second Real input signal"
        annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
      Blocks.Interfaces.BooleanOutput y "Connector of Boolean output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              fillColor={210,210,210},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised),
            Ellipse(
              extent={{73,7},{87,-7}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid),
            Ellipse(extent={{32,10},{52,-10}}, lineColor={0,0,127}),
            Line(points={{-100,-80},{42,-80},{42,0}}, color={0,0,127}),
                                                   Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255})}),
          Documentation(info="<html>
<p>
Block has two continuous Real input and one continuous Boolean output signal
as a result of the comparison of the two input signals. The block
has a 3D icon (e.g., used in Blocks.Logical library).
</p>
</html>"));

    end partialBooleanComparison;

    partial block PartialBooleanSISO_small
      "Partial block with a BooleanInput and a BooleanOutput signal and a small block icon"

      Blocks.Interfaces.BooleanInput u "Boolean input signal"
        annotation (Placement(transformation(extent={{-180,-40},{-100,40}})));
      Blocks.Interfaces.BooleanOutput y "Boolean output signal"
        annotation (Placement(transformation(extent={{100,-20},{140,20}})));
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            initialScale=0.04), graphics={
            Text(
              extent={{-300,200},{300,120}},
              textString="%name",
              textColor={0,0,255}),
            Rectangle(
              extent={{-100,100},{100,-100}},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised),
            Ellipse(
              extent={{60,10},{80,-10}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid)}));
    end PartialBooleanSISO_small;

    partial block PartialBooleanMISO
      "Partial block with a BooleanVectorInput and a BooleanOutput signal"

      parameter Integer nu(min=0) = 0 "Number of input connections"
        annotation (Dialog(connectorSizing=true), HideResult=true);
      Blocks.Interfaces.BooleanVectorInput u[nu]
        "Vector of Boolean input signals"
        annotation (Placement(transformation(extent={{-120,70},{-80,-70}})));
      Blocks.Interfaces.BooleanOutput y "Boolean output signal"
        annotation (Placement(transformation(extent={{100,-15},{130,15}})));
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}},
            initialScale=0.06), graphics={
            Text(
              extent={{-250,170},{250,110}},
              textString="%name",
              textColor={0,0,255}),
            Rectangle(
              extent={{-100,100},{100,-100}},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised),
            Ellipse(
              extent={{60,10},{80,-10}},
              lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
              fillPattern=FillPattern.Solid)}));
    end PartialBooleanMISO;

    partial block PartialConversionBlock
      "Partial block defining the interface for conversion blocks"

      RealInput u "Connector of Real input signal to be converted" annotation (
          Placement(transformation(extent={{-140,-20},{-100,20}})));
      RealOutput y
        "Connector of Real output signal containing input signal u in another unit"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}),
            graphics={
          Rectangle(
            lineColor={0,0,127},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            extent={{-100.0,-100.0},{100.0,100.0}}),
          Line(
            points={{-90.0,0.0},{30.0,0.0}},
            color={191,0,0}),
          Polygon(
            lineColor={191,0,0},
            fillColor={191,0,0},
            fillPattern=FillPattern.Solid,
            points={{90.0,0.0},{30.0,20.0},{30.0,-20.0},{90.0,0.0}}),
          Text(
            textColor={0,0,255},
            extent={{-150,110},{150,150}},
            textString="%name")}), Documentation(info="<html>
<p>
This block defines the interface of a conversion block that
converts from one unit into another one.
</p>

</html>"));

    end PartialConversionBlock;

    partial block PartialNoise "Partial noise generator"
      import generator = Modelica.Math.Random.Generators.Xorshift128plus;
      import Modelica.Math.Random.Utilities.automaticLocalSeed;
      extends Blocks.Interfaces.SO;

      // Main dialog menu
      parameter SI.Period samplePeriod(start=0.01)
        "Period for sampling the raw random numbers"
        annotation(Dialog(enable=enableNoise));

      // Advanced dialog menu: Noise generation
      parameter Boolean enableNoise = globalSeed.enableNoise
        "= true: y = noise, otherwise y = y_off"
        annotation(choices(checkBox=true),Dialog(tab="Advanced",group="Noise generation"));
      parameter Real y_off = 0.0
        "Sets y = y_off if enableNoise=false (or time<startTime, see below)"
        annotation(Dialog(tab="Advanced",group="Noise generation"));

      // Advanced dialog menu: Initialization
      parameter Boolean useGlobalSeed = true
        "= true: use global seed, otherwise ignore it"
        annotation(choices(checkBox=true),Dialog(tab="Advanced",group = "Initialization",enable=enableNoise));
      parameter Boolean useAutomaticLocalSeed = true
        "= true: use automatic local seed, otherwise use fixedLocalSeed"
        annotation(choices(checkBox=true),Dialog(tab="Advanced",group = "Initialization",enable=enableNoise));
      parameter Integer fixedLocalSeed = 1 "Local seed (any Integer number)"
        annotation(Dialog(tab="Advanced",group = "Initialization",enable=enableNoise and not useAutomaticLocalSeed));
      parameter SI.Time startTime = 0.0
        "Start time for sampling the raw random numbers"
        annotation(Dialog(tab="Advanced", group="Initialization",enable=enableNoise));
      final parameter Integer localSeed(fixed=false) "The actual localSeed";
    protected
      outer Blocks.Noise.GlobalSeed globalSeed
        "Definition of global seed via inner/outer";
      parameter Integer actualGlobalSeed = if useGlobalSeed then globalSeed.seed else 0
        "The global seed, which is actually used";
      parameter Boolean generateNoise = enableNoise and globalSeed.enableNoise
        "= true, if noise shall be generated, otherwise no noise";

      // Declare state and random number variables
      Integer state[generator.nState] "Internal state of random number generator";
      discrete Real r "Random number according to the desired distribution";
      discrete Real r_raw "Uniform random number in the range (0,1]";

    initial equation
       localSeed = if useAutomaticLocalSeed then automaticLocalSeed(getInstanceName()) else fixedLocalSeed;
       pre(state) = generator.initialState(localSeed, actualGlobalSeed);
       r_raw = generator.random(pre(state));

    equation
      // Draw random number at sample times
      when generateNoise and sample(startTime, samplePeriod) then
        (r_raw, state) = generator.random(pre(state));
      end when;

      // Generate noise if requested
      y = if not generateNoise or time < startTime then y_off else r;

        annotation(Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics={
            Polygon(
              points={{-76,90},{-84,68},{-68,68},{-76,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-76,68},{-76,-80}}, color={192,192,192}),
            Line(points={{-86,-14},{72,-14}},
                                          color={192,192,192}),
            Polygon(
              points={{94,-14},{72,-6},{72,-22},{94,-14}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(visible = enableNoise,
               points={{-76,-19},{-62,-19},{-62,-3},{-54,-3},{-54,-51},{-46,-51},{-46,
                  -29},{-38,-29},{-38,55},{-30,55},{-30,23},{-30,23},{-30,-37},{-20,
                  -37},{-20,-19},{-10,-19},{-10,-47},{0,-47},{0,35},{6,35},{6,49},{12,
                  49},{12,-7},{22,-7},{22,5},{28,5},{28,-25},{38,-25},{38,47},{48,47},
                  {48,13},{56,13},{56,-53},{66,-53}}),
            Text(
              extent={{-150,-110},{150,-150}},
              textString="%samplePeriod s"),
            Line(visible=not enableNoise,
              points={{-76,48},{72,48}}),
            Text(visible=not enableNoise,
              extent={{-75,42},{95,2}},
              textString="%y_off"),
            Text(visible=enableNoise and not useAutomaticLocalSeed,
              extent={{-92,20},{98,-22}},
              textColor={238,46,47},
              textString="%fixedLocalSeed")}),
        Documentation(info="<html>
<p>
Partial base class of noise generators defining the common features
of noise blocks.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end PartialNoise;

    package Adaptors "Package with adaptors (especially useful for FMUs)"
      extends Modelica.Icons.InterfacesPackage;

      partial model FlowToPotentialAdaptor "Signal adaptor for a connector with flow, 1st derivative of flow, and 2nd derivative of flow as inputs and
  potential, 1st derivative of potential, and 2nd derivative of potential as outputs (especially useful for FMUs)"
        parameter Boolean use_pder=true "Use output for 1st derivative of potential"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        parameter Boolean use_pder2=true "Use output for 2nd derivative of potential (only if 1st derivative is used, too)"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        parameter Boolean use_fder=true "Use input for 1st derivative of flow"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        parameter Boolean use_fder2=true "Use input for 2nd derivative of flow (only if 1st derivative is used, too)"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        Blocks.Interfaces.RealOutput p "Output for potential"
          annotation (Placement(transformation(extent={{20,70},{40,90}})));
        Blocks.Interfaces.RealOutput pder if use_pder
          "Optional output for der(potential)"
          annotation (Placement(transformation(extent={{20,40},{40,60}})));
        Blocks.Interfaces.RealOutput pder2 if (use_pder and use_pder2)
          "Optional output for der2(potential)"
          annotation (Placement(transformation(extent={{20,10},{40,30}})));
        Blocks.Interfaces.RealInput f "Input for flow"
          annotation (Placement(transformation(extent={{40,-90},{20,-70}})));
        Blocks.Interfaces.RealInput fder if use_fder
          "Optional input for der(flow)"
          annotation (Placement(transformation(extent={{40,-60},{20,-40}})));
        Blocks.Interfaces.RealInput fder2 if (use_fder and use_fder2)
          "Optional input for der2(flow)"
          annotation (Placement(transformation(extent={{40,-30},{20,-10}})));
      protected
        parameter String Name_p="p" "Name of potential variable";
        parameter String Name_pder="der(p)" "Name of 1st derivative of potential variable";
        parameter String Name_pder2="der2(p)" "Name of 2nd derivative of potential variable";
        parameter String Name_f="f" "Name of flow variable";
        parameter String Name_fder="der(f)" "Name of 1st derivative of flow variable";
        parameter String Name_fder2="der2(f)" "Name of 2nd derivative of flow variable";
        Real y "Output signal" annotation(HideResult=true);
        Blocks.Interfaces.RealOutput y1 "Optional 1st derivative of output"
          annotation (HideResult=true);
        Blocks.Interfaces.RealOutput y2 "Optional 2nd derivative of output"
          annotation (HideResult=true);
        Real u "Input signal" annotation(HideResult=true);
        Blocks.Interfaces.RealInput u1 "Optional 1st derivative of input"
          annotation (HideResult=true);
        Blocks.Interfaces.RealInput u2 "Optional 2nd derivative of input"
          annotation (HideResult=true);
      equation
        y = p;
        y1 = if use_pder then der(y) else 0;
        y2 = if (use_pder and use_pder2) then der(y1) else 0;
        connect(y1, pder);
        connect(y2, pder2);
        if use_fder then
          connect(fder, u1);
        else
          u1 = 0;
        end if;
        if (use_fder and use_fder2) then
          connect(fder2, u2);
        else
          u2 = 0;
        end if;
        if (use_fder and use_fder2) then
          u = Functions.state2({f, u1, u2}, time);
        elseif (use_fder and not use_fder2) then
          u = Functions.state1({f, u1}, time);
        else
          u = f;
        end if;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Text(
                extent={{-150,150},{150,110}},
                textColor={0,0,255},
                textString="%name"),
                  Rectangle(
                    extent={{-20,100},{20,-100}},
                    lineColor={0,0,127},
                    radius=10),
              Text(
                extent={{-18,90},{18,70}},
                textString="%Name_p"),
              Text(
                extent={{-18,60},{18,40}},
                textString="%Name_pder",
                visible=use_pder),
              Text(
                extent={{-18,30},{18,10}},
                textString="%Name_pder2",
                visible=(use_pder and use_pder2)),
              Text(
                extent={{-18,-70},{18,-90}},
                textString="%Name_f"),
              Text(
                extent={{-18,-40},{18,-60}},
                textString="%Name_fder",
                visible=use_fder),
              Text(
                extent={{-18,-10},{18,-30}},
                textString="%Name_fder2",
                visible=(use_fder and use_fder2))}),
                Diagram(coordinateSystem(preserveAspectRatio=false)),
          Documentation(info="<html>
<p>
Adaptor between a physical connector and a signal representation of the connector signals.
This component is used to provide a pure signal interface around a physical model
and export this model in form of an input/output block,
especially as FMU (<a href=\"https://fmi-standard.org\">Functional Mock-up Unit</a>).
</p>
<p>
This adaptor has flow, optional 1st derivative of flow, and optional 2nd derivative of flow as input and
potential, optional 1st derivative of potential, and optional 2nd derivative of potential as output signals.
</p>
<p>
Note, the input signals must be consistent to each other
(fder=der(f), fder2=der(fder)).
</p>
</html>"));
      end FlowToPotentialAdaptor;

      partial model PotentialToFlowAdaptor "Signal adaptor for a connector with potential, 1st derivative of potential, and 2nd derivative of potential as inputs and
  flow, 1st derivative of flow, and 2nd derivative of flow as outputs (especially useful for FMUs)"
        parameter Boolean use_pder=true "Use input for 1st derivative of potential"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        parameter Boolean use_pder2=true "Use input for 2nd derivative of potential (only if 1st derivative is used, too)"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        parameter Boolean use_fder=true "Use output for 1st derivative of flow"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        parameter Boolean use_fder2=true "Use output for 2nd derivative of flow (only if 1st derivative is used, too)"
          annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
        Blocks.Interfaces.RealInput p "Input for potential"
          annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
        Blocks.Interfaces.RealInput pder if use_pder
          "Optional input for der(potential)"
          annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
        Blocks.Interfaces.RealInput pder2 if (use_pder and use_pder2)
          "Optional input for der2(potential)"
          annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
        Blocks.Interfaces.RealOutput f "Output for flow"
          annotation (Placement(transformation(extent={{-20,-90},{-40,-70}})));
        Blocks.Interfaces.RealOutput fder if use_fder
          "Optional output for der(flow)"
          annotation (Placement(transformation(extent={{-20,-60},{-40,-40}})));
        Blocks.Interfaces.RealOutput fder2 if (use_fder and use_fder2)
          "Optional output for der2(flow)"
          annotation (Placement(transformation(extent={{-20,-30},{-40,-10}})));
      protected
        parameter String Name_p="p" "Name of potential variable";
        parameter String Name_pder="der(p)" "Name of 1st derivative of potential variable";
        parameter String Name_pder2="der2(p)" "Name of 2nd derivative of potential variable";
        parameter String Name_f="f" "Name of flow variable";
        parameter String Name_fder="der(f)" "Name of 1st derivative of flow variable";
        parameter String Name_fder2="der2(f)" "Name of 2nd derivative of flow variable";
        Real y "Output signal" annotation(HideResult=true);
        Blocks.Interfaces.RealOutput y1 "Optional 1st derivative of output"
          annotation (HideResult=true);
        Blocks.Interfaces.RealOutput y2 "Optional 2nd derivative of output"
          annotation (HideResult=true);
        Real u "Input signal" annotation(HideResult=true);
        Blocks.Interfaces.RealInput u1 "Optional 1st derivative of input"
          annotation (HideResult=true);
        Blocks.Interfaces.RealInput u2 "Optional 2nd derivative of input"
          annotation (HideResult=true);
      equation
        y = -f;
        y1 = if use_fder then -der(y) else 0;
        y2 = if (use_fder and use_fder2) then -der(y1) else 0;
        connect(y1, fder);
        connect(y2, fder2);
        if use_pder then
          connect(pder, u1);
        else
          u1 = 0;
        end if;
        if (use_pder and use_pder2) then
          connect(pder2, u2);
        else
          u2 = 0;
        end if;
        if (use_pder and use_pder2) then
          u = Functions.state2({p, u1, u2}, time);
        elseif (use_pder and not use_pder2) then
          u = Functions.state1({p, u1}, time);
        else
          u = p;
        end if;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Text(
                extent={{-150,150},{150,110}},
                textColor={0,0,255},
                textString="%name"),
                  Rectangle(
                    extent={{-20,100},{20,-100}},
                    lineColor={0,0,127},
                    radius=10),
              Text(
                extent={{-18,90},{18,70}},
                textString="%Name_p"),
              Text(
                extent={{-18,60},{18,40}},
                textString="%Name_pder",
                visible=use_pder),
              Text(
                extent={{-18,30},{18,10}},
                textString="%Name_pder2",
                visible=(use_pder and use_pder2)),
              Text(
                extent={{-18,-70},{18,-90}},
                textString="%Name_f"),
              Text(
                extent={{-18,-40},{18,-60}},
                textString="%Name_fder",
                visible=use_fder),
              Text(
                extent={{-18,-10},{18,-30}},
                textString="%Name_fder2",
                visible=(use_fder and use_fder2))}),
                Diagram(coordinateSystem(preserveAspectRatio=false)),
          Documentation(info="<html>
<p>
Adaptor between a physical connector and a signal representation of the connector signals.
This component is used to provide a pure signal interface around a physical model
and export this model in form of an input/output block,
especially as FMU (<a href=\"https://fmi-standard.org\">Functional Mock-up Unit</a>).
</p>
<p>
This adaptor has potential, optional 1st derivative of potential, and optional 2nd derivative of potential as input and
flow, optional 1st derivative of flow, and optional 2nd derivative of flow as output signals.
</p>
<p>
Note, the input signals must be consistent to each other
(pder=der(p), pder2=der(pder)).
</p>
</html>"));
      end PotentialToFlowAdaptor;

      package Functions "Functions for adaptors"
        extends Modelica.Icons.FunctionsPackage;

        function state1 "Return state (with one derivative)"
          extends Modelica.Icons.Function;
          input Real u[2] "Required values for state and der(s)";
          input Real dummy
            "Just to have one input signal that should be differentiated to avoid possible problems in the Modelica tool (is not used)";
          output Real s;
        algorithm
          s := u[1];
          annotation (derivative(noDerivative=u) = state1der1,
              InlineAfterIndexReduction=true);
        end state1;

        function state1der1 "Return 1st derivative (der of state1)"
          extends Modelica.Icons.Function;
          input Real u[2] "Required values for state and der(s)";
          input Real dummy
            "Just to have one input signal that should be differentiated to avoid possible problems in the Modelica tool (is not used)";
          input Real dummy_der;
          output Real sder1;
        algorithm
          sder1 := u[2];
          annotation (InlineAfterIndexReduction=true);
        end state1der1;

        function state2 "Return state (with two derivatives)"
          extends Modelica.Icons.Function;
          input Real u[3] "Required values for state and der(s)";
          input Real dummy
            "Just to have one input signal that should be differentiated to avoid possible problems in the Modelica tool (is not used)";
          output Real s;
        algorithm
          s := u[1];
          annotation (derivative(noDerivative=u) = state2der1,
              InlineAfterIndexReduction=true);
        end state2;

        function state2der1 "Return 1st derivative (der of state2)"
          extends Modelica.Icons.Function;
          input Real u[3] "Required values for state and der(s)";
          input Real dummy
            "Just to have one input signal that should be differentiated to avoid possible problems in the Modelica tool (is not used)";
          input Real dummy_der;
          output Real sder1;
        algorithm
          sder1 := u[2];
          annotation (derivative(noDerivative=u, order=2) = state2der2,
              InlineAfterIndexReduction=true);
        end state2der1;

        function state2der2 "Return 2nd derivative (der of state2der1)"
          extends Modelica.Icons.Function;
          input Real u[3] "Required values for state and der(s)";
          input Real dummy
            "Just to have one input signal that should be differentiated to avoid possible problems in the Modelica tool (is not used)";
          input Real dummy_der;
          input Real dummy_der2;
          output Real sder2;
        algorithm
          sder2 := u[3];
          annotation (InlineAfterIndexReduction=true);
        end state2der2;
      end Functions;

      annotation (Documentation(info="<html>
<p>
This package contains partial adaptors to implement adaptors in various domains
between a physical connector and a signal representation of the connector signals.
This component is used to provide a pure signal interface around a physical model
and export this model in form of an input/output block,
especially as FMU (<a href=\"https://fmi-standard.org\">Functional Mock-up Unit</a>).
</p>
</html>"));
    end Adaptors;
    annotation (Documentation(info="<html>
<p>
This package contains interface definitions for
<strong>continuous</strong> input/output blocks with Real,
Integer and Boolean signals. Furthermore, it contains
partial models for continuous and discrete blocks.
</p>

</html>",   revisions="<html>
<ul>
<li><em>June 28, 2019</em>
       by Thomas Beutlich:<br>
       Removed obsolete blocks.</li>
<li><em>Oct. 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       Added several new interfaces.</li>
<li><em>Oct. 24, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       RealInputSignal renamed to RealInput. RealOutputSignal renamed to
       output RealOutput. GraphBlock renamed to BlockIcon. SISOreal renamed to
       SISO. SOreal renamed to SO. I2SOreal renamed to M2SO.
       SignalGenerator renamed to SignalSource. Introduced the following
       new models: MIMO, MIMOs, SVcontrol, MVcontrol, DiscreteBlockIcon,
       DiscreteBlock, DiscreteSISO, DiscreteMIMO, DiscreteMIMOs,
       BooleanBlockIcon, BooleanSISO, BooleanSignalSource, MI2BooleanMOs.</li>
<li><em>June 30, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"));
  end Interfaces;

  package Logical "Library of components with Boolean input and output signals"
    extends Modelica.Icons.Package;

    block And "Logical 'and': y = u1 and u2"
      extends Blocks.Interfaces.partialBooleanSI2SO;
    equation
      y = u1 and u2;
      annotation (
        defaultComponentName="and1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="and")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if all inputs are <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end And;

    block Or "Logical 'or': y = u1 or u2"
      extends Blocks.Interfaces.partialBooleanSI2SO;
    equation
      y = u1 or u2;
      annotation (
        defaultComponentName="or1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="or")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if at least one input is <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end Or;

    block Xor "Logical 'xor': y = u1 xor u2"
      extends Blocks.Interfaces.partialBooleanSI2SO;
    equation
      y = not ((u1 and u2) or (not u1 and not u2));
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="xor")}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if exactly one input is <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end Xor;

    block Nor "Logical 'nor': y = not (u1 or u2)"
      extends Blocks.Interfaces.partialBooleanSI2SO;
    equation
      y = not (u1 or u2);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="nor")}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if none of the inputs is <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end Nor;

    block Nand "Logical 'nand': y = not (u1 and u2)"
      extends Blocks.Interfaces.partialBooleanSI2SO;
    equation
      y = not (u1 and u2);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="nand")}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if at least one input is <strong>false</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end Nand;

    block Not "Logical 'not': y = not u"
      extends Blocks.Interfaces.partialBooleanSISO;

    equation
      y = not u;
      annotation (
        defaultComponentName="not1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="not")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if the input is <strong>false</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end Not;

    block Pre
      "Breaks algebraic loops by an infinitesimal small time delay (y = pre(u): event iteration continues until u = pre(u))"

      parameter Boolean pre_u_start=false "Start value of pre(u) at initial time";
      extends Blocks.Interfaces.partialBooleanSISO;

    initial equation
      pre(u) = pre_u_start;
    equation
      y = pre(u);
      annotation (
        defaultComponentName="pre1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="pre")}),
        Documentation(info="<html>
<p>
This block delays the Boolean input by an infinitesimal small time delay and
therefore breaks algebraic loops. In a network of logical blocks, in every
\"closed connection loop\" at least one logical block must have a delay,
since algebraic systems of Boolean equations are not solvable.
</p>

<p>
The \"Pre\" block returns the value of the \"input\" signal from the
last \"event iteration\". The \"event iteration\" stops, once both
values are identical (u = pre(u)).
</p>
</html>"));
    end Pre;

    block Edge "Output y is true, if the input u has a rising edge (y = edge(u))"

      parameter Boolean pre_u_start=false "Start value of pre(u) at initial time";
      extends Blocks.Interfaces.partialBooleanSISO;

    initial equation
      pre(u) = pre_u_start;
    equation
      y = edge(u);
      annotation (
        defaultComponentName="edge1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="edge")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Boolean input has a rising edge
from <strong>false</strong> to <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end Edge;

    block FallingEdge
      "Output y is true, if the input u has a falling edge (y = edge(not u))"

      parameter Boolean pre_u_start=false "Start value of pre(u) at initial time";
      extends Blocks.Interfaces.partialBooleanSISO;

    protected
      Boolean not_u=not u;
    initial equation
      pre(not_u) = not pre_u_start;
    equation
      y = edge(not_u);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="falling")}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Boolean input has a falling edge
from <strong>true</strong> to <strong>false</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end FallingEdge;

    block Change
      "Output y is true, if the input u has a rising or falling edge (y = change(u))"

      parameter Boolean pre_u_start=false "Start value of pre(u) at initial time";
      extends Blocks.Interfaces.partialBooleanSISO;

    initial equation
      pre(u) = pre_u_start;
    equation
      y = change(u);
      annotation (
        defaultComponentName="change1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Text(
              extent={{-90,40},{90,-40}},
              textString="change")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Boolean input has either a rising edge
from <strong>false</strong> to <strong>true</strong> or a falling edge from
<strong>true</strong> to <strong>false</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end Change;

    block GreaterThreshold
      "Output y is true, if input u is greater than threshold"
      extends Blocks.Interfaces.partialBooleanThresholdComparison;
    equation
      y = u > threshold;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Line(
              points={{-54,20},{-8,0},{-54,-20}},
              thickness=0.5)}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Real input is greater than
parameter <strong>threshold</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end GreaterThreshold;

    block GreaterEqualThreshold
      "Output y is true, if input u is greater or equal than threshold"

      extends Blocks.Interfaces.partialBooleanThresholdComparison;
    equation
      y = u >= threshold;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Line(
              points={{-54,20},{-8,0},{-54,-20}},
              thickness=0.5),
            Line(points={{-54,-30},{-8,-30}}, thickness=0.5)}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Real input is greater than or equal to
parameter <strong>threshold</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end GreaterEqualThreshold;

    block LessThreshold "Output y is true, if input u is less than threshold"

      extends Blocks.Interfaces.partialBooleanThresholdComparison;
    equation
      y = u < threshold;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Line(points={{-8,20},{-54,0},{-8,-20}}, thickness=0.5)}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Real input is less than
parameter <strong>threshold</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end LessThreshold;

    block LessEqualThreshold
      "Output y is true, if input u is less or equal than threshold"
      extends Blocks.Interfaces.partialBooleanThresholdComparison;
    equation
      y = u <= threshold;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Line(points={{-8,20},{-54,0},{-8,-20}}, thickness=0.5),
            Line(points={{-54,-30},{-8,-30}}, thickness=0.5)}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Real input is less than or equal to
parameter <strong>threshold</strong>, otherwise
the output is <strong>false</strong>.
</p>
</html>"));
    end LessEqualThreshold;

    block Greater "Output y is true, if input u1 is greater than input u2"
      extends Blocks.Interfaces.partialBooleanComparison;

    equation
      y = u1 > u2;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                -100},{100,100}}), graphics={
            Ellipse(extent={{32,10},{52,-10}}, lineColor={0,0,127}),
            Line(points={{-100,-80},{42,-80},{42,0}}, color={0,0,127}),
            Line(
              points={{-54,20},{-8,0},{-54,-20}},
              thickness=0.5)}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if Real input u1 is greater than
Real input u2, otherwise the output is <strong>false</strong>.
</p>
</html>"));
    end Greater;

    block GreaterEqual
      "Output y is true, if input u1 is greater or equal than input u2"
      extends Blocks.Interfaces.partialBooleanComparison;

    equation
      y = u1 >= u2;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                -100},{100,100}}), graphics={
            Ellipse(extent={{32,10},{52,-10}}, lineColor={0,0,127}),
            Line(points={{-100,-80},{42,-80},{42,0}}, color={0,0,127}),
            Line(
              points={{-54,20},{-8,0},{-54,-20}},
              thickness=0.5),
            Line(points={{-54,-30},{-8,-30}}, thickness=0.5)}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if Real input u1 is greater than or equal to
Real input u2, otherwise the output is <strong>false</strong>.
</p>
</html>"));
    end GreaterEqual;

    block Less "Output y is true, if input u1 is less than input u2"
      extends Blocks.Interfaces.partialBooleanComparison;

    equation
      y = u1 < u2;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                -100},{100,100}}), graphics={
            Ellipse(extent={{32,10},{52,-10}}, lineColor={0,0,127}),
            Line(points={{-100,-80},{42,-80},{42,0}}, color={0,0,127}),
            Line(points={{-8,20},{-54,0},{-8,-20}}, thickness=0.5)}), Documentation(info="<html>
<p>
The output is <strong>true</strong> if Real input u1 is less than
Real input u2, otherwise the output is <strong>false</strong>.
</p>
</html>"));
    end Less;

    block LessEqual "Output y is true, if input u1 is less or equal than input u2"
      extends Blocks.Interfaces.partialBooleanComparison;

    equation
      y = u1 <= u2;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                -100},{100,100}}), graphics={
            Ellipse(extent={{32,10},{52,-10}}, lineColor={0,0,127}),
            Line(points={{-100,-80},{42,-80},{42,0}}, color={0,0,127}),
            Line(points={{-8,20},{-54,0},{-8,-20}}, thickness=0.5),
            Line(points={{-54,-30},{-8,-30}}, thickness=0.5)}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if Real input u1 is less than or equal to
Real input u2, otherwise the output is <strong>false</strong>.
</p>
</html>"));
    end LessEqual;

    block ZeroCrossing "Trigger zero crossing of input u"
      extends Blocks.Interfaces.partialBooleanSO;
      Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.BooleanInput enable
        "Zero input crossing is triggered if the enable input signal is true"
        annotation (Placement(transformation(
            origin={0,-120},
            extent={{-20,-20},{20,20}},
            rotation=90)));

    protected
      Boolean disable=not enable;
      Boolean u_pos;
    initial equation
      pre(u_pos) = false;
      pre(enable) = false;
      pre(disable) = not pre(enable);
    equation
      u_pos = enable and u >= 0;
      y = change(u_pos) and not edge(enable) and not edge(disable);
      annotation (Documentation(info="<html>
<p>
The output \"y\" is <strong>true</strong> at the
time instant when the input \"u\" becomes
zero, provided the input \"enable\" is
<strong>true</strong>. At all other time instants, the output \"y\" is <strong>false</strong>.
If the input \"u\" is zero at a time instant when the \"enable\"
input changes its value, then the output y is <strong>false</strong>.
</p>
<p>
Note, that in the plot window of a Modelica simulator, the output of
this block is usually identically to <strong>false</strong>, because the output
may only be <strong>true</strong> at an event instant, but not during
continuous integration. In order to check that this component is
actually working as expected, one should connect its output to, e.g.,
component <em><a href=\"modelica://Modelica.Blocks.Discrete.TriggeredSampler\">Modelica.Blocks.Discrete.TriggeredSampler</a></em>.
</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Line(points={{-78,68},{-78,-80}}, color={192,192,192}),
            Polygon(
              points={{-78,90},{-86,68},{-70,68},{-78,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-88,0},{70,0}}, color={192,192,192}),
            Line(points={{-78,0},{-73.2,32.3},{-70,50.3},{-66.7,64.5},{-63.5,74.2},
                  {-60.3,79.3},{-57.1,79.6},{-53.9,75.3},{-50.7,67.1},{-46.6,52.2},
                  {-41,25.8},{-33,-13.9},{-28.2,-33.7},{-24.1,-45.9},{-20.1,-53.2},
                  {-16.1,-55.3},{-12.1,-52.5},{-8.1,-45.3},{-3.23,-32.1},{10.44,
                  13.7},{15.3,26.4},{20.1,34.8},{24.1,38},{28.9,37.2},{33.8,31.8},
                  {40.2,19.4},{53.1,-10.5},{59.5,-21.2},{65.1,-25.9},{70.7,-25.9},
                  {77.2,-20.5},{82,-13.8}}, color={192,192,192}, smooth = Smooth.Bezier),
            Polygon(
              points={{92,0},{70,8},{70,-8},{92,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-36,-59},{-36,81}}, color={255,0,255}),
            Line(points={{6,-59},{6,81}}, color={255,0,255}),
            Line(points={{49,-59},{49,81}}, color={255,0,255}),
            Line(points={{-78,0},{70,0}}, color={255,0,255})}));
    end ZeroCrossing;

    block LogicalSwitch "Logical Switch"
      extends Blocks.Interfaces.partialBooleanSI3SO;

    equation
      y = if u2 then u1 else u3;
      annotation (Documentation(info="<html>
<p>The LogicalSwitch switches, depending on the
Boolean u2 connector (the middle connector),
between the two possible input signals
u1 (upper connector) and u3 (lower connector).</p>
<p>If u2 is true, connector y is set equal to
u1, else it is set equal to u3.</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
              graphics={
            Line(
              points={{12,0},{100,0}},
              color={255,0,255}),
            Line(
              points={{-100,0},{-40,0}},
              color={255,0,255}),
            Line(
              points={{-100,-80},{-40,-80},{-40,-80}},
              color={255,0,255}),
            Line(points={{-40,12},{-40,-10}}, color={255,0,255}),
            Line(points={{-100,80},{-40,80}}, color={255,0,255}),
            Line(
              points=DynamicSelect({{-40,80},{8,2}}, if u2 then {{-40,80},{8,2}} else {{-40,-80},{8,2}}),
              color={255,0,255},
              thickness=1),
            Ellipse(lineColor={0,0,127},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{2,-6},{18,8}})}));
    end LogicalSwitch;

    block Switch "Switch between two Real signals"
      extends Blocks.Icons.PartialBooleanBlock;
      Blocks.Interfaces.RealInput u1 "Connector of first Real input signal"
        annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
      Blocks.Interfaces.BooleanInput u2 "Connector of Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealInput u3 "Connector of second Real input signal"
        annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
      Blocks.Interfaces.RealOutput y "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      y = if u2 then u1 else u3;
      annotation (
        defaultComponentName="switch1",
        Documentation(info="<html>
<p>The Logical.Switch switches, depending on the
logical connector u2 (the middle connector)
between the two possible input signals
u1 (upper connector) and u3 (lower connector).</p>
<p>If u2 is <strong>true</strong>, the output signal y is set equal to
u1, else it is set equal to u3.</p>
</html>"),
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{12,0},{100,0}},
              color={0,0,127}),
            Line(points={{-100,0},{-40,0}},
              color={255,0,255}),
            Line(points={{-100,-80},{-40,-80},{-40,-80}},
              color={0,0,127}),
            Line(points={{-40,12},{-40,-12}},
              color={255,0,255}),
            Line(points={{-100,80},{-38,80}},
              color={0,0,127}),
            Line(points=DynamicSelect({{-38,80},{6,2}}, if u2 then {{-38,80},{6,2}} else {{-38,-80},{6,2}}),
              color={0,0,127},
              thickness=1),
            Ellipse(lineColor={0,0,255},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{2,-8},{18,8}})}));
    end Switch;

    block Hysteresis "Transform Real to Boolean signal with Hysteresis"

      extends Blocks.Icons.PartialBooleanBlock;
      parameter Real uLow(start=0) "If y=true and u<uLow, switch to y=false";
      parameter Real uHigh(start=1) "If y=false and u>uHigh, switch to y=true";
      parameter Boolean pre_y_start=false "Value of pre(y) at initial time";

      Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.BooleanOutput y
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    initial equation
      pre(y) = pre_y_start;
    equation
      assert(uHigh > uLow,"Hysteresis limits wrong (uHigh <= uLow)");
      y = not pre(y) and u > uHigh or pre(y) and u >= uLow;
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Polygon(
                points={{-65,89},{-73,67},{-57,67},{-65,89}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),Line(points={{-65,67},{-65,-81}},
              color={192,192,192}),Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
              Polygon(
                points={{90,-70},{68,-62},{68,-78},{90,-70}},
                lineColor={192,192,192},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),Text(
                extent={{70,-80},{94,-100}},
                textColor={160,160,164},
                textString="u"),Text(
                extent={{-65,93},{-12,75}},
                textColor={160,160,164},
                textString="y"),Line(
                points={{-80,-70},{30,-70}},
                thickness=0.5),Line(
                points={{-50,10},{80,10}},
                thickness=0.5),Line(
                points={{-50,10},{-50,-70}},
                thickness=0.5),Line(
                points={{30,10},{30,-70}},
                thickness=0.5),Line(
                points={{-10,-65},{0,-70},{-10,-75}},
                thickness=0.5),Line(
                points={{-10,15},{-20,10},{-10,5}},
                thickness=0.5),Line(
                points={{-55,-20},{-50,-30},{-44,-20}},
                thickness=0.5),Line(
                points={{25,-30},{30,-19},{35,-30}},
                thickness=0.5),Text(
                extent={{-99,2},{-70,18}},
                textColor={160,160,164},
                textString="true"),Text(
                extent={{-98,-87},{-66,-73}},
                textColor={160,160,164},
                textString="false"),Text(
                extent={{19,-87},{44,-70}},
                textString="uHigh"),Text(
                extent={{-63,-88},{-38,-71}},
                textString="uLow"),Line(points={{-69,10},{-60,10}}, color={160,
              160,164})}),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,68},{-80,-29}}, color={192,192,192}),
            Polygon(
              points={{92,-29},{70,-21},{70,-37},{92,-29}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-79,-29},{84,-29}}, color={192,192,192}),
            Line(points={{-79,-29},{41,-29}}),
            Line(points={{-15,-21},{1,-29},{-15,-36}}),
            Line(points={{41,51},{41,-29}}),
            Line(points={{33,3},{41,22},{50,3}}),
            Line(points={{-49,51},{81,51}}),
            Line(points={{-4,59},{-19,51},{-4,43}}),
            Line(points={{-59,29},{-49,11},{-39,29}}),
            Line(points={{-49,51},{-49,-29}}),
            Text(
              extent={{-92,-49},{-9,-92}},
              textColor={192,192,192},
              textString="%uLow"),
            Text(
              extent={{2,-49},{91,-92}},
              textColor={192,192,192},
              textString="%uHigh"),
            Rectangle(extent={{-91,-49},{-8,-92}}, lineColor={192,192,192}),
            Line(points={{-49,-29},{-49,-49}}, color={192,192,192}),
            Rectangle(extent={{2,-49},{91,-92}}, lineColor={192,192,192}),
            Line(points={{41,-29},{41,-49}}, color={192,192,192})}),
        Documentation(info="<html>
<p>
This block transforms a <strong>Real</strong> input signal <strong>u</strong> into a <strong>Boolean</strong>
output signal <strong>y</strong>:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Logical/Hysteresis.png\"
     alt=\"Hysteresis.png\">
</p>

<ul>
<li> When the output was <strong>false</strong> and the input becomes
     <strong>greater</strong> than parameter <strong>uHigh</strong>, the output
     switches to <strong>true</strong>.</li>
<li> When the output was <strong>true</strong> and the input becomes
     <strong>less</strong> than parameter <strong>uLow</strong>, the output
     switches to <strong>false</strong>.</li>
</ul>
<p>
The start value of the output is defined via parameter
<strong>pre_y_start</strong> (= value of pre(y) at initial time).
The default value of this parameter is <strong>false</strong>.
</p>
</html>"));
    end Hysteresis;

    block OnOffController "On-off controller"
      extends Blocks.Icons.PartialBooleanBlock;
      Blocks.Interfaces.RealInput reference
        "Connector of Real input signal used as reference signal"
        annotation (Placement(transformation(extent={{-140,80},{-100,40}})));
      Blocks.Interfaces.RealInput u
        "Connector of Real input signal used as measurement signal"
        annotation (Placement(transformation(extent={{-140,-40},{-100,-80}})));
      Blocks.Interfaces.BooleanOutput y
        "Connector of Real output signal used as actuator signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      parameter Real bandwidth(start=0.1) "Bandwidth around reference signal";
      parameter Boolean pre_y_start=false "Value of pre(y) at initial time";

    initial equation
      pre(y) = pre_y_start;
    equation
      y = pre(y) and (u < reference + bandwidth/2) or (u < reference - bandwidth/
        2);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={
            Text(
              extent={{-92,74},{44,44}},
              textString="reference"),
            Text(
              extent={{-94,-52},{-34,-74}},
              textString="u"),
            Line(points={{-76,-32},{-68,-6},{-50,26},{-24,40},{-2,42},{16,36},{32,28},{48,12},{58,-6},{68,-28}},
              color={0,0,127}),
            Line(points={{-78,-2},{-6,18},{82,-12}},
              color={255,0,0}),
            Line(points={{-78,12},{-6,30},{82,0}}),
            Line(points={{-78,-16},{-6,4},{82,-26}}),
            Line(points={{-82,-18},{-56,-18},{-56,-40},{64,-40},{64,-20},{90,-20}},
              color={255,0,255})}), Documentation(info="<html>
<p>The block OnOffController sets the output signal <strong>y</strong> to <strong>true</strong> when
the input signal <strong>u</strong> falls below the <strong>reference</strong> signal minus half of
the bandwidth and sets the output signal <strong>y</strong> to <strong>false</strong> when the input
signal <strong>u</strong> exceeds the <strong>reference</strong> signal plus half of the bandwidth.</p>
</html>"));
    end OnOffController;

    block TriggeredTrapezoid "Triggered trapezoid generator"
      extends Blocks.Icons.PartialBooleanBlock;

      parameter Real amplitude=1 "Amplitude of trapezoid";
      parameter SI.Time rising(final min=0) = 0
        "Rising duration of trapezoid";
      parameter SI.Time falling(final min=0) = rising
        "Falling duration of trapezoid";
      parameter Real offset=0 "Offset of output signal";

      Blocks.Interfaces.BooleanInput u "Connector of Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    protected
      discrete Real endValue "Value of y at time of recent edge";
      discrete Real rate "Current rising/falling rate";
      discrete SI.Time T
        "Predicted time of output reaching endValue";
    equation
      y = if time < T then endValue - (T - time)*rate else endValue;

      when {initial(),u,not u} then
        endValue = if u then offset + amplitude else offset;
        rate = if u and (rising > 0) then amplitude/rising else if not u and (
          falling > 0) then -amplitude/falling else 0;
        T = if u and not (rising > 0) or not u and not (falling > 0) or not abs(
          amplitude) > 0 or initial() then time else time + (endValue - pre(y))/
          rate;
      end when;
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
            graphics={
          Line(points={{-60,-70},{-60,-70},{-30,40},{8,40},{40,-70},{40,-70}},
            color={0,0,127}),
          Line(points={{-90,-70},{82,-70}},
            color={192,192,192}),
          Line(points={{-80,68},{-80,-80}},
            color={192,192,192}),
          Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{90,-70},{68,-62},{68,-78},{90,-70}}),
          Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{-80,90},{-88,68},{-72,68},{-80,90}}),
          Line(points={{-80,-70},{-60,-70},{-60,24},{8,24},{8,-70},{60,-70}},
            color={255,0,255})}),
        Documentation(info="<html>
<p>The block TriggeredTrapezoid has a Boolean input and a real
output signal and requires the parameters <em>amplitude</em>,
<em>rising</em>, <em>falling</em> and <em>offset</em>. The
output signal <strong>y</strong> represents a trapezoidal signal dependent on the
input signal <strong>u</strong>.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Logical/TriggeredTrapezoid.png\"
     alt=\"TriggeredTrapezoid.png\">
</p>

<p>The behaviour is as follows: Assume the initial input to be false. In this
case, the output will be <em>offset</em>. After a rising edge (i.e., the input
changes from false to true), the output is rising during <em>rising</em> to the
sum of <em>offset</em> and <em>amplitude</em>. In contrast, after a falling
edge (i.e., the input changes from true to false), the output is falling
during <em>falling</em> to a value of <em>offset</em>.
</p>
<p>Note, that the case of edges before expiration of rising or falling is
handled properly.</p>
</html>"));
    end TriggeredTrapezoid;

    block Timer
      "Timer measuring the time from the time instant where the Boolean input became true"

      extends Blocks.Icons.PartialBooleanBlock;
      Blocks.Interfaces.BooleanInput u "Connector of Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    protected
      discrete SI.Time entryTime "Time instant when u became true";
    initial equation
      pre(entryTime) = 0;
    equation
      when u then
        entryTime = time;
      end when;
      y = if u then time - entryTime else 0.0;
      annotation (
        Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
            graphics={
          Line(points={{-90,-70},{82,-70}},
            color={192,192,192}),
          Line(points={{-80,68},{-80,-80}},
            color={192,192,192}),
          Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{90,-70},{68,-62},{68,-78},{90,-70}}),
          Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{-80,90},{-88,68},{-72,68},{-80,90}}),
          Line(points={{-80,-70},{-60,-70},{-60,-26},{38,-26},{38,-70},{66,-70}},
            color={255,0,255}),
          Line(points={{-80,0},{-62,0},{40,90},{40,0},{68,0}},
            color={0,0,127})}),
        Documentation(info="<html>
<p>When the Boolean input <strong>u</strong> becomes <strong>true</strong>, the timer starts
and the output <strong>y</strong> is the time that has elapsed since <strong>u</strong> became <strong>true</strong>.
When the input becomes <strong>false</strong>, the timer stops and the output is reset to zero.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Logical/Timer.png\"
     alt=\"Timer.png\">
</p>

</html>"));
    end Timer;

    block LogicalDelay "Delay boolean signal"
      extends Blocks.Icons.PartialBooleanBlock;
      parameter SI.Time delayTime(final min=0)=0 "Time delay";
      Blocks.Interfaces.BooleanInput u
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.BooleanOutput y1
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
      Blocks.Interfaces.BooleanOutput y2
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
    protected
      discrete SI.Time tSwitch;
    initial equation
      tSwitch = time - 2*delayTime;
    equation
      when {u, not u} then
        tSwitch = time;
      end when;
      y1 = if u then true else not (time >= tSwitch + delayTime);
      y2 = if not u then false else (time >= tSwitch + delayTime);
      annotation (Documentation(info="<html>
<p>
When input <code>u</code> gets true, output <code>y1</code> gets immediately true, whereas output <code>y2</code> gets true after <code>delayTime</code>.
</p>
<p>
When input <code>u</code> gets false, output <code>y1</code> gets false after <code>delayTime</code>, whereas output <code>y2</code> gets immediately false.
</p>
</html>"),   Icon(graphics={
          Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{-80,90},{-88,68},{-72,68},{-80,90}}),
          Line(points={{-80,68},{-80,-80}},
            color={192,192,192}),
          Line(points={{-90,-70},{82,-70}},
            color={192,192,192}),
          Polygon(lineColor={192,192,192},
            fillColor={192,192,192},
            fillPattern=FillPattern.Solid,
            points={{90,-70},{68,-62},{68,-78},{90,-70}}),
            Line(points={{-80,-10},{-60,-10},{-60,10},{40,10},{40,-10},{80,-10}},
                color={255,0,255}),
            Line(points={{-80,50},{-60,50},{-60,70},{50,70},{50,50},{80,50}},
                color={255,0,255}),
            Line(points={{-80,-70},{-50,-70},{-50,-50},{40,-50},{40,-70},{80,-70}},
                color={255,0,255}),
            Line(
              points={{-60,70},{-60,-70}},
              color={192,192,192},
              pattern=LinePattern.Dot),
            Line(
              points={{40,70},{40,-70}},
              color={192,192,192},
              pattern=LinePattern.Dot)}));
    end LogicalDelay;

    block RSFlipFlop "A basic RS Flip Flop"
      extends Blocks.Icons.PartialBooleanBlock;
      parameter Boolean Qini=false "Start value of Q at initial time";
      Blocks.Interfaces.BooleanOutput Q
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
      Blocks.Interfaces.BooleanOutput QI
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
      Blocks.Logical.Nor nor
        annotation (Placement(transformation(extent={{-20,20},{0,40}})));
      Blocks.Logical.Nor nor1
        annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
      Blocks.Logical.Pre pre(pre_u_start=not (Qini))
        annotation (Placement(transformation(extent={{10,20},{30,40}})));
      Blocks.Interfaces.BooleanInput S
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Blocks.Interfaces.BooleanInput R
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
    equation
      connect(nor1.y, nor.u2) annotation (Line(points={{1,-10},{40,-10},{40,-40},
              {-60,-40},{-60,22},{-22,22}}, color={255,0,255}));
      connect(nor1.y, Q) annotation (Line(points={{1,-10},{60,-10},{60,60},{110,
              60}}, color={255,0,255}));
      connect(nor.y, pre.u)
        annotation (Line(points={{1,30},{8,30}}, color={255,0,255}));
      connect(pre.y, nor1.u1) annotation (Line(points={{31,30},{40,30},{40,10},{-40,
              10},{-40,-10},{-22,-10}}, color={255,0,255}));
      connect(pre.y, QI) annotation (Line(points={{31,30},{80,30},{80,-60},{110,-60}}, color={255,0,255}));
      connect(S, nor.u1) annotation (Line(
          points={{-120,60},{-40,60},{-40,30},{-22,30}}, color={255,0,255}));
      connect(R, nor1.u2) annotation (Line(
          points={{-120,-60},{-40,-60},{-40,-18},{-22,-18}}, color={255,0,255}));
    annotation (
      Icon(graphics={
          Text(
            extent={{-60,-30},{-20,-90}},
            textString="R"),
          Text(
            extent={{-62,90},{-22,30}},
            textString="S"),
          Text(
            extent={{20,90},{60,30}},
            textString="Q"),
          Text(
            extent={{6,-30},{66,-90}},
            textString="Q!"),
          Ellipse(
            extent={{-73,54},{-87,68}},
            lineColor=DynamicSelect({235,235,235}, if S then {0,255,0} else {235,235,235}),
            fillColor=DynamicSelect({235,235,235}, if S then {0,255,0} else {235,235,235}),
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{83,-53},{69,-67}},
            lineColor=DynamicSelect({235,235,235}, if QI then {0,255,0} else {235,235,235}),
            fillColor=DynamicSelect({235,235,235}, if QI then {0,255,0} else {235,235,235}),
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-71,-52},{-85,-66}},
            lineColor=DynamicSelect({235,235,235}, if R then {0,255,0} else {235,235,235}),
            fillColor=DynamicSelect({235,235,235}, if R then {0,255,0} else {235,235,235}),
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{71,67},{85,53}},
            lineColor=DynamicSelect({235,235,235}, if Q then {0,255,0} else {235,235,235}),
            fillColor=DynamicSelect({235,235,235}, if Q then {0,255,0} else {235,235,235}),
            fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
The output <code>Q</code> is set by the input <code>S</code>, is reset by the input <code>R</code>, and keeps its value in between. <code>QI</code> is the inverse of <code>Q</code>.
</p>
</html>"));
    end RSFlipFlop;

    block TerminateSimulation "Terminate simulation if condition is fulfilled"

      Blocks.Interfaces.BooleanOutput condition=false
        "Terminate simulation when condition becomes true" annotation (Dialog,
          Placement(transformation(extent={{200,-10},{220,10}})));
      parameter String terminationText="... End condition reached"
        "Text that will be displayed when simulation is terminated";

    equation
      when condition then
        terminate(terminationText);
      end when;
      annotation (Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-200,-20},{200,20}},
            initialScale=0.2),
            graphics={
          Rectangle(fillColor={235,235,235},
            fillPattern=FillPattern.Solid,
            lineThickness=5,
            borderPattern=BorderPattern.Raised,
            extent={{-200,-20},{200,20}}),
          Text(extent={{-166,-15},{194,15}},
            textString="%condition"),
          Rectangle(fillColor={161,35,41},
            fillPattern=FillPattern.Solid,
            borderPattern=BorderPattern.Raised,
            extent={{-194,-14},{-168,14}}),
          Text(textColor={0,0,255},
            extent={{-200,22},{200,46}},
            textString="%name")}), Documentation(info="<html>
<p>
In the parameter menu, a <strong>time varying</strong> expression can be defined
via variable <strong>condition</strong>, for example \"condition = x &lt; 0\",
where \"x\" is a variable that is declared in the model in which the
\"TerminateSimulation\" block is present.
If this expression becomes <strong>true</strong>,
the simulation is (successfully) terminated. A termination message
explaining the reason for the termination can be given via
parameter \"terminationText\".
</p>

</html>"));
    end TerminateSimulation;
    annotation (Documentation(info="<html>
<p>
This package provides blocks with Boolean input and output signals
to describe logical networks. A typical example for a logical
network built with package Logical is shown in the next figure:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Logical/LogicalNetwork1.png\"
     alt=\"LogicalNetwork1.png\">
</p>

<p>
The actual value of Boolean input and/or output signals is displayed
in the respective block icon as \"circle\", where \"white\" color means
value <strong>false</strong> and \"green\" color means value <strong>true</strong>. These
values are visualized in a diagram animation.
</p>
</html>"),   Icon(graphics={Line(
            points={{-86,-22},{-50,-22},{-50,22},{48,22},{48,-22},{88,-24}},
            color={255,0,255})}));
  end Logical;

  package Math "Library of Real mathematical functions as input/output blocks"

    import Blocks.Interfaces;
    extends Modelica.Icons.Package;

    encapsulated package UnitConversions
      "Conversion blocks to convert between SI and non-SI unit signals"
      import Modelica;

      import Modelica.Units.NonSI;
      extends Modelica.Icons.Package;

      block To_degC "Convert from Kelvin to degCelsius"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="K"), y(
              unit="degC"));

      equation
        y = Modelica.Units.Conversions.to_degC(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="K"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="degC")}), Documentation(info="<html>
<p>
This block converts the input signal from Kelvin to degCelsius and returns
the result as output signal.
</p>
</html>"));
      end To_degC;

      block From_degC "Convert from degCelsius to Kelvin"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="degC"),
            y(unit="K"));
      equation
        y = Modelica.Units.Conversions.from_degC(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="degC"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="K")}), Documentation(info="<html>
<p>
This block converts the input signal from degCelsius to Kelvin and returns
the result as output signal.
</p>
</html>"));
      end From_degC;

      block To_degF "Convert from Kelvin to degFahrenheit"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="K"), y(
              unit="degF"));
      equation
        y = Modelica.Units.Conversions.to_degF(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="K"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="degF")}), Documentation(info="<html>
<p>
This block converts the input signal from Kelvin to degFahrenheit and returns
the result as output signal.
</p>
</html>"));
      end To_degF;

      block From_degF "Convert from degFahrenheit to Kelvin"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="degF"),
            y(unit="K"));
      equation
        y = Modelica.Units.Conversions.from_degF(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="degF"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="K")}), Documentation(info="<html>
<p>
This block converts the input signal from degFahrenheit to Kelvin and returns
the result as output signal.
</p>
</html>"));
      end From_degF;

      block To_degRk "Convert from Kelvin to degRankine"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="K"), y(
              unit="degRk"));
      equation
        y = Modelica.Units.Conversions.to_degRk(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="K"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="degRk")}), Documentation(info="<html>
<p>
This block converts the input signal from Kelvin to degRankine and returns
the result as output signal.
</p>
</html>"));
      end To_degRk;

      block From_degRk "Convert from degRankine to Kelvin"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="degRk"),
            y(unit="K"));
      equation
        y = Modelica.Units.Conversions.from_degRk(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="degRk"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="K")}), Documentation(info="<html>
<p>
This block converts the input signal from degRankine to Kelvin and returns
the result as output signal.
</p>
</html>"));
      end From_degRk;

      block To_deg "Convert from radian to degree"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="rad"),
            y(unit="deg"));
      equation
        y = Modelica.Units.Conversions.to_deg(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="rad"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="deg")}), Documentation(info="<html>
<p>
This block converts the input signal from radian to degree and returns
the result as output signal.
</p>
</html>"));
      end To_deg;

      block From_deg "Convert from degree to radian"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="deg"),
            y(unit="rad"));
      equation
        y = Modelica.Units.Conversions.from_deg(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="deg"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="rad")}), Documentation(info="<html>
<p>
This block converts the input signal from degree to radian and returns
the result as output signal.
</p>
</html>"));
      end From_deg;

      block To_rpm "Convert from radian per second to revolutions per minute"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="rad/s"),
            y(unit="rev/min"));
      equation
        y = Modelica.Units.Conversions.to_rpm(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{26,82},{-98,50}},
                    textString="rad/s"),Text(
                    extent={{100,-42},{-62,-74}},
                    textString="rev/min")}), Documentation(info="<html>
<p>
This block converts the input signal from radian per second to revolutions per minute and returns
the result as output signal.
</p>
</html>"));
      end To_rpm;

      block From_rpm "Convert from revolutions per minute to radian per second"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="rev/min"),
            y(unit="rad/s"));
      equation
        y = Modelica.Units.Conversions.from_rpm(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{50,84},{-94,56}},
                    textString="rev/min"),Text(
                    extent={{94,-42},{-26,-74}},
                    textString="rad/s")}), Documentation(info="<html>
<p>
This block converts the input signal from revolutions per minute to radian per second and returns
the result as output signal.
</p>
</html>"));
      end From_rpm;

      block To_kmh "Convert from metre per second to kilometre per hour"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="m/s"),
            y(unit="km/h"));
      equation
        y = Modelica.Units.Conversions.to_kmh(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{0,82},{-96,42}},
                    textString="m/s"),Text(
                    extent={{92,-40},{-14,-84}},
                    textString="km/h")}), Documentation(info="<html>
<p>
This block converts the input signal from metre per second to kilometre per hour and returns
the result as output signal.
</p>
</html>"));
      end To_kmh;

      block From_kmh "Convert from kilometre per hour to metre per second"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="km/h"),
            y(unit="m/s"));
      equation
        y = Modelica.Units.Conversions.from_kmh(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{26,80},{-96,48}},
                    textString="km/h"),Text(
                    extent={{92,-46},{-20,-82}},
                    textString="m/s")}), Documentation(info="<html>
<p>
This block converts the input signal from kilometre per hour to metre per second and returns
the result as output signal.
</p>
</html>"));
      end From_kmh;

      block To_day "Convert from second to day"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="s"), y(
              unit="d"));
      equation
        y = Modelica.Units.Conversions.to_day(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="s"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="day")}), Documentation(info="<html>
<p>
This block converts the input signal from second to day and returns
the result as output signal.
</p>
</html>"));
      end To_day;

      block From_day "Convert from day to second"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="d"), y(
              unit="s"));
      equation
        y = Modelica.Units.Conversions.from_day(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="day"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="s")}), Documentation(info="<html>
<p>
This block converts the input signal from day to second and returns
the result as output signal.
</p>
</html>"));
      end From_day;

      block To_hour "Convert from second to hour"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="s"), y(
              unit="h"));
      equation
        y = Modelica.Units.Conversions.to_hour(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="s"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="hour")}), Documentation(info="<html>
<p>
This block converts the input signal from second to hour and returns
the result as output signal.
</p>
</html>"));
      end To_hour;

      block From_hour "Convert from hour to second"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="h"), y(
              unit="s"));
      equation
        y = Modelica.Units.Conversions.from_hour(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="hour"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="s")}), Documentation(info="<html>
<p>
This block converts the input signal from hour to second and returns
the result as output signal.
</p>
</html>"));
      end From_hour;

      block To_minute "Convert from second to minute"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="s"), y(
              unit="min"));
      equation
        y = Modelica.Units.Conversions.to_minute(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="s"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="minute")}), Documentation(info="<html>
<p>
This block converts the input signal from second to minute and returns
the result as output signal.
</p>
</html>"));
      end To_minute;

      block From_minute "Convert from minute to second"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="min"),
            y(unit="s"));
      equation
        y = Modelica.Units.Conversions.from_minute(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="minute"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="s")}), Documentation(info="<html>
<p>
This block converts the input signal from minute to second and returns
the result as output signal.
</p>
</html>"));
      end From_minute;

      block To_litre "Convert from cubic metre to litre"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="m3"), y(
              unit="l"));
      equation
        y = Modelica.Units.Conversions.to_litre(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="m3"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="litre")}), Documentation(info="<html>
<p>
This block converts the input signal from metre to litre and returns
the result as output signal.
</p>
</html>"));
      end To_litre;

      block From_litre "Convert from litre to cubic metre"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="l"), y(
              unit="m3"));
      equation
        y = Modelica.Units.Conversions.from_litre(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="litre"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="m3")}), Documentation(info="<html>
<p>
This block converts the input signal from litre to cubic metre and returns
the result as output signal.
</p>
</html>"));
      end From_litre;

      block To_kWh "Convert from Joule to kilo Watt hour"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="J"), y(
              unit="kW.h"));
      equation
        y = Modelica.Units.Conversions.to_kWh(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="J"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="kW.h")}), Documentation(info="<html>
<p>
This block converts the input signal from Joule to kilo Watt hour and returns
the result as output signal.
</p>
</html>"));
      end To_kWh;

      block From_kWh "Convert from kilo Watt hour to Joule"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="kW.h"),
            y(unit="J"));
      equation
        y = Modelica.Units.Conversions.from_kWh(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="kW.h"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="J")}), Documentation(info="<html>
<p>
This block converts the input signal from kilo Watt hour to Joule and returns
the result as output signal.
</p>
</html>"));
      end From_kWh;

      block To_bar "Convert from Pascal to bar"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="Pa"), y(
              unit="bar"));
      equation
        y = Modelica.Units.Conversions.to_bar(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="Pa"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="bar")}), Documentation(info="<html>
<p>
This block converts the input signal from Pascal to bar and returns
the result as output signal.
</p>
</html>"));
      end To_bar;

      block From_bar "Convert from bar to Pascal"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="bar"),
            y(unit="Pa"));
      equation
        y = Modelica.Units.Conversions.from_bar(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="bar"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="Pa")}), Documentation(info="<html>
<p>
This block converts the input signal from bar to Pascal and returns
the result as output signal.
</p>
</html>"));
      end From_bar;

      block To_gps "Convert from kilogram per second to gram per second"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="kg/s"),
            y(unit="g/s"));
      equation
        y = Modelica.Units.Conversions.to_gps(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="kg/s"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="g/s")}), Documentation(info="<html>
<p>
This block converts the input signal from kilogram per second to gram per seconds and returns
the result as output signal.
</p>
</html>"));
      end To_gps;

      block From_gps "Convert from gram per second to kilogram per second"
        extends Modelica.Blocks.Interfaces.PartialConversionBlock(u(unit="g/s"),
            y(unit="kg/s"));
      equation
        y = Modelica.Units.Conversions.from_gps(u);
        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                  -100},{100,100}}), graphics={Text(
                    extent={{-20,100},{-100,20}},
                    textString="g/s"),Text(
                    extent={{100,-20},{20,-100}},
                    textString="kg/s")}), Documentation(info="<html>
<p>
This block converts the input signal from gram per second to kilogram per second and returns
the result as output signal.
</p>
</html>"));
      end From_gps;
      annotation (Documentation(info="<html>
<p>
This package consists of blocks that convert an input signal
with a specific unit to an output signal in another unit
(e.g., conversion of an angle signal from \"deg\" to \"rad\").
</p>

</html>"));
    end UnitConversions;

    block InverseBlockConstraints
      "Construct inverse model by requiring that two inputs and two outputs are identical"

      Blocks.Interfaces.RealInput u1 "Input signal 1 (u1 = u2)" annotation (
          Placement(transformation(extent={{-240,-20},{-200,20}}),
            iconTransformation(extent={{-240,-20},{-200,20}})));
      Blocks.Interfaces.RealInput u2 "Input signal 2 (u1 = u2)" annotation (
          Placement(transformation(extent={{-140,-20},{-180,20}}),
            iconTransformation(extent={{-140,-20},{-180,20}})));
      Blocks.Interfaces.RealOutput y1 "Output signal 1 (y1 = y2)" annotation (
          Placement(transformation(extent={{200,-10},{220,10}}),
            iconTransformation(extent={{200,-10},{220,10}})));
      Blocks.Interfaces.RealOutput y2 "Output signal 2 (y1 = y2)" annotation (
          Placement(transformation(extent={{10,-10},{-10,10}}, origin={170,0}),
            iconTransformation(extent={{180,-10},{160,10}})));

    equation
      u1 = u2;
      y1 = y2;
      annotation (
        defaultConnectionStructurallyInconsistent=true,
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-120},{200,
                120}}), graphics={
            Line(
              points={{180,0},{200,0}},
              color={0,0,127}),
            Line(
              points={{-200,0},{-180,0}},
              color={0,0,127}),
            Rectangle(extent={{-190,120},{190,-120}}, lineColor={135,135,135})}),
        Documentation(info="<html>
<p>
Exchange input and output signals of a block, i.e., the previous
block inputs become block outputs and the previous block outputs become
block inputs. This block is used to construct inverse models.
Its usage is demonstrated in example:
<a href=\"modelica://Modelica.Blocks.Examples.InverseModel\">Modelica.Blocks.Examples.InverseModel</a>.
</p>

<p>
Note, if a block shall be inverted that has several input and output blocks,
then this can be easily achieved by using a vector of InverseBlockConstraints
instances:
</p>

<blockquote><pre>
InverseBlockConstraint invert[3];  // Block to be inverted has 3 input signals
</pre></blockquote>
</html>"));
    end InverseBlockConstraints;

    block Gain "Output the product of a gain value with the input signal"

      parameter Real k(start=1, unit="1")
        "Gain value multiplied with input signal";
    public
      Interfaces.RealInput u "Input signal connector" annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.RealOutput y "Output signal connector" annotation (Placement(
            transformation(extent={{100,-10},{120,10}})));

    equation
      y = k*u;
      annotation (
        Documentation(info="<html>
<p>
This block computes output <em>y</em> as
<em>product</em> of gain <em>k</em> with the
input <em>u</em>:
</p>
<blockquote><pre>
y = k * u;
</pre></blockquote>

</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{-100,-100},{-100,100},{100,0},{-100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-150,-140},{150,-100}},
              textString="k=%k"),
            Text(
              extent={{-150,140},{150,100}},
              textString="%name",
              textColor={0,0,255})}));
    end Gain;

    block MatrixGain
      "Output the product of a gain matrix with the input signal vector"

      parameter Real K[:, :]=[1, 0; 0, 1]
        "Gain matrix which is multiplied with the input";
      extends Interfaces.MIMO(final nin=size(K, 2), final nout=size(K, 1));
    equation
      y = K*u;
      annotation (
        Documentation(info="<html>
<p>
This blocks computes output vector <strong>y</strong> as <em>product</em> of the
gain matrix <strong>K</strong> with the input signal vector <strong>u</strong>:
</p>
<blockquote><pre>
<strong>y</strong> = <strong>K</strong> * <strong>u</strong>;
</pre></blockquote>
<p>
Example:
</p>
<blockquote><pre>
parameter: <strong>K</strong> = [0.12 2; 3 1.5]

results in the following equations:

  | y[1] |     | 0.12  2.00 |   | u[1] |
  |      |  =  |            | * |      |
  | y[2] |     | 3.00  1.50 |   | u[2] |
</pre></blockquote>

</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Text(
              extent={{-90,-60},{90,60}},
              textColor={160,160,164},
              textString="*K")}));
    end MatrixGain;

    block MultiSum "Sum of Reals: y = k[1]*u[1] + k[2]*u[2] + ... + k[n]*u[n]"
      extends Blocks.Interfaces.PartialRealMISO;
      parameter Real k[nu]=fill(1, nu) "Input gains";
    equation
      if size(u, 1) > 0 then
        y = k*u;
      else
        y = 0;
      end if;

      annotation (Icon(graphics={Text(
              extent={{-200,-110},{200,-140}},
              textString="%k"), Text(
              extent={{-72,68},{92,-68}},
              textString="+")}), Documentation(info="<html>
<p>
This blocks computes the scalar Real output \"y\" as sum of the elements of the
Real input signal vector u:
</p>
<blockquote><pre>
y = k[1]*u[1] + k[2]*u[2] + ... k[N]*u[N];
</pre></blockquote>

<p>
The input connector is a vector of Real input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.RealNetwork1\">Modelica.Blocks.Examples.RealNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to zero: y=0.
</p>

</html>"));
    end MultiSum;

    block MultiProduct "Product of Reals: y = u[1]*u[2]* ... *u[n]"
      extends Blocks.Interfaces.PartialRealMISO;
    equation
      if size(u, 1) > 0 then
        y = product(u);
      else
        y = 0;
      end if;

      annotation (Icon(graphics={Text(
              extent={{-74,50},{94,-94}},
              textString="*")}), Documentation(info="<html>
<p>
This blocks computes the scalar Real output \"y\" as product of the elements of the
Real input signal vector u:
</p>
<blockquote><pre>
y = u[1]*u[2]* ... *u[N];
</pre></blockquote>

<p>
The input connector is a vector of Real input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.RealNetwork1\">Modelica.Blocks.Examples.RealNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to zero: y=0.
</p>
</html>"));
    end MultiProduct;

    block MultiSwitch
      "Set Real expression that is associated with the first active input signal"

      input Real expr[nu]=fill(0.0, nu)
        "y = if u[i] then expr[i] else y_default (time varying)"
        annotation (Dialog);
      parameter Real y_default=0.0
        "Default value of output y if all u[i] = false";

      parameter Integer nu(min=0) = 0 "Number of input connections"
        annotation (Dialog(connectorSizing=true), HideResult=true);
      parameter Integer precision(min=0) = 3
        "Number of significant digits to be shown in dynamic diagram layer for y"
        annotation (Dialog(tab="Advanced"));

      Blocks.Interfaces.BooleanVectorInput u[nu]
        "Set y = expr[i], if u[i] = true"
        annotation (Placement(transformation(extent={{-110,30},{-90,-30}})));
      Blocks.Interfaces.RealOutput y "Output depending on expression"
        annotation (Placement(transformation(extent={{300,-10},{320,10}})));

    protected
      Integer firstActiveIndex;
    initial equation
      pre(u) = fill(false, nu);
    equation
      firstActiveIndex = Modelica.Math.BooleanVectors.firstTrueIndex(u);
      y = if firstActiveIndex == 0 then y_default else expr[firstActiveIndex];
      annotation (
        defaultComponentName="multiSwitch1",
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{300,
                100}}), graphics={
            Rectangle(
              extent={{-100,-51},{300,50}},
              lineThickness=5.0,
              fillColor={170,213,255},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised),
            Text(
              extent={{-86,16},{295,-17}},
              textString="%expr"),
            Text(
              extent={{310,-25},{410,-45}},
              textString=DynamicSelect(" ", String(
                    y,
                    minimumLength=1,
                    significantDigits=precision))),
            Text(
              extent={{-100,-60},{300,-90}},
              textString="else: %y_default"),
            Text(
              extent={{-100,100},{300,60}},
              textString="%name",
              textColor={0,0,255})}),
        Documentation(info="<html>
<p>
This block has a vector of Boolean input signals u[nu] and a vector of
(time varying) Real expressions expr[nu]. The output signal y is
set to expr[i], if i is the first element in the input vector u that is true. If all input signals are
false, y is set to parameter \"y_default\".
</p>

<blockquote><pre>
// Conceptual equation (not valid Modelica)
i = 'first element of u[:] that is true';
y = <strong>if</strong> i==0 <strong>then</strong> y_default <strong>else</strong> expr[i];
</pre></blockquote>

<p>
The input connector is a vector of Boolean input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.RealNetwork1\">Modelica.Blocks.Examples.RealNetwork1</a>.
</p>

</html>"));
    end MultiSwitch;

    block Sum "Output the sum of the elements of the input vector"
      extends Interfaces.MISO;
      parameter Real k[nin]=ones(nin) "Optional: sum coefficients";
    equation
      y = k*u;
      annotation (
        defaultComponentName="sum1",
        Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as
<em>sum</em> of the elements of the input signal vector
<strong>u</strong>:
</p>
<blockquote><pre>
<strong>y</strong> = <strong>u</strong>[1] + <strong>u</strong>[2] + ...;
</pre></blockquote>
<p>
Example:
</p>
<blockquote><pre>
   parameter:   nin = 3;

results in the following equations:

   y = u[1] + u[2] + u[3];
</pre></blockquote>

</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Line(
              points={{26,42},{-34,42},{6,2},{-34,-38},{26,-38}})}));
    end Sum;

    block Feedback "Output difference between commanded and feedback input"

      Interfaces.RealInput u1 "Commanded input" annotation (Placement(transformation(extent={{-100,
                -20},{-60,20}})));
      Interfaces.RealInput u2 "Feedback input" annotation (Placement(transformation(
            origin={0,-80},
            extent={{-20,-20},{20,20}},
            rotation=90)));
      Interfaces.RealOutput y annotation (Placement(transformation(extent={{80,-10},
                {100,10}})));

    equation
      y = u1 - u2;
      annotation (
        Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as <em>difference</em> of the
commanded input <strong>u1</strong> and the feedback
input <strong>u2</strong>:
</p>
<blockquote><pre>
<strong>y</strong> = <strong>u1</strong> - <strong>u2</strong>;
</pre></blockquote>
<p>
Example:
</p>
<blockquote><pre>
   parameter:   n = 2

results in the following equations:

   y = u1 - u2
</pre></blockquote>

</html>"),
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              lineColor={0,0,127},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              extent={{-20,-20},{20,20}}),
            Line(points={{-60,0},{-20,0}}, color={0,0,127}),
            Line(points={{20,0},{80,0}}, color={0,0,127}),
            Line(points={{0,-20},{0,-60}}, color={0,0,127}),
            Text(extent={{-14,-94},{82,0}}, textString="-"),
            Text(
              textColor={0,0,255},
              extent={{-150,40},{150,80}},
              textString="%name")}));
    end Feedback;

    block Add "Output the sum of the two inputs"
      extends Interfaces.SI2SO;

      parameter Real k1=+1 "Gain of input signal 1";
      parameter Real k2=+1 "Gain of input signal 2";

    equation
      y = k1*u1 + k2*u2;
      annotation (
        Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as <em>sum</em> of the
two input signals <strong>u1</strong> and <strong>u2</strong>:
</p>
<blockquote><pre>
<strong>y</strong> = k1*<strong>u1</strong> + k2*<strong>u2</strong>;
</pre></blockquote>
<p>
Example:
</p>
<blockquote><pre>
   parameter:   k1= +2, k2= -3

results in the following equations:

   y = 2 * u1 - 3 * u2
</pre></blockquote>

</html>"),
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-100,60},{-74,24},{-44,24}}, color={0,0,127}),
            Line(points={{-100,-60},{-74,-24},{-44,-24}}, color={0,0,127}),
            Ellipse(lineColor={0,0,127}, extent={{-50,-50},{50,50}}),
            Line(points={{50,0},{100,0}}, color={0,0,127}),
            Text(extent={{-40,40},{40,-40}}, textString="+"),
            Text(extent={{-100,52},{5,92}}, textString="%k1"),
            Text(extent={{-100,-92},{5,-52}}, textString="%k2")}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={         Line(points={{50,0},{100,0}},
              color={0,0,255}),                                        Line(
              points={{50,0},{100,0}}, color={0,0,127})}));
    end Add;

    block Add3 "Output the sum of the three inputs"
      extends Blocks.Icons.Block;

      parameter Real k1=+1 "Gain of input signal 1";
      parameter Real k2=+1 "Gain of input signal 2";
      parameter Real k3=+1 "Gain of input signal 3";
      Interfaces.RealInput u1 "Connector of Real input signal 1" annotation (
          Placement(transformation(extent={{-140,60},{-100,100}})));
      Interfaces.RealInput u2 "Connector of Real input signal 2" annotation (
          Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.RealInput u3 "Connector of Real input signal 3" annotation (
          Placement(transformation(extent={{-140,-100},{-100,-60}})));
      Interfaces.RealOutput y "Connector of Real output signal" annotation (
          Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      y = k1*u1 + k2*u2 + k3*u3;
      annotation (
        Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as <em>sum</em> of the
three input signals <strong>u1</strong>, <strong>u2</strong> and <strong>u3</strong>:
</p>
<blockquote><pre>
<strong>y</strong> = k1*<strong>u1</strong> + k2*<strong>u2</strong> + k3*<strong>u3</strong>;
</pre></blockquote>
<p>
Example:
</p>
<blockquote><pre>
   parameter:   k1= +2, k2= -3, k3=1;

results in the following equations:

   y = 2 * u1 - 3 * u2 + u3;
</pre></blockquote>

</html>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Text(
              extent={{-100,50},{5,90}},
              textString="%k1"),
            Text(
              extent={{-100,-20},{5,20}},
              textString="%k2"),
            Text(
              extent={{-100,-50},{5,-90}},
              textString="%k3"),
            Text(
              extent={{10,40},{90,-40}},
              textString="+")}));
    end Add3;

    block Product "Output product of the two inputs"
      extends Interfaces.SI2SO;

    equation
      y = u1*u2;
      annotation (
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <em>product</em> of the two inputs <strong>u1</strong> and <strong>u2</strong>:
</p>
<blockquote><pre>
y = u1 * u2;
</pre></blockquote>

</html>"),
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-100,60},{-40,60},{-30,40}}, color={0,0,127}),
            Line(points={{-100,-60},{-40,-60},{-30,-40}}, color={0,0,127}),
            Line(points={{50,0},{100,0}}, color={0,0,127}),
            Line(points={{-30,0},{30,0}}),
            Line(points={{-15,25.99},{15,-25.99}}),
            Line(points={{-15,-25.99},{15,25.99}}),
            Ellipse(lineColor={0,0,127}, extent={{-50,-50},{50,50}})}));
    end Product;

    block Division "Output first input divided by second input"
      extends Interfaces.SI2SO;

    equation
      y = u1/u2;
      annotation (
        Documentation(info="<html>
<p>
This block computes the output <strong>y</strong>
by <em>dividing</em> the two inputs <strong>u1</strong> and <strong>u2</strong>:
</p>
<blockquote><pre>
y = u1 / u2;
</pre></blockquote>

</html>"),
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-100,60},{-60,60},{0,0}}, color={0,0,127}),
            Line(points={{-100,-60},{-60,-60},{0,0}}, color={0,0,127}),
            Ellipse(lineColor={0,0,127}, extent={{-50,-50},{50,50}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{50,0},{100,0}}, color={0,0,127}),
            Line(points={{-30,0},{30,0}}),
            Ellipse(fillPattern=FillPattern.Solid, extent={{-5,20},{5,30}}),
            Ellipse(fillPattern=FillPattern.Solid, extent={{-5,-30},{5,-20}}),
            Text(
              extent={{-60,90},{90,50}},
              textColor={128,128,128},
              textString="u1 / u2")}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={         Line(points={{50,0},{100,0}},
              color={0,0,255})}));
    end Division;

    block Abs "Output the absolute value of the input"
      extends Interfaces.SISO;
      parameter Boolean generateEvent=false
        "Choose whether events shall be generated" annotation (Evaluate=true);
    equation
      //y = abs(u);
      y = if generateEvent then (if u >= 0 then u else -u) else (if noEvent(u >=
        0) then u else -u);
      annotation (
        defaultComponentName="abs1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{92,0},{70,8},{70,-8},{92,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,80},{0,0},{80,80}}),
            Line(points={{0,-14},{0,68}}, color={192,192,192}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-34,-28},{38,-76}},
              textColor={192,192,192},
              textString="abs"),
            Line(points={{-88,0},{76,0}}, color={192,192,192})}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <em>absolute value</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>abs</strong>( u );
</pre></blockquote>
<p>
The Boolean parameter generateEvent decides whether Events are generated at zero crossing (Modelica specification before 3) or not.
</p>
</html>"));
    end Abs;

    block Sign "Output the sign of the input"
      extends Interfaces.SISO;
      parameter Boolean generateEvent=false
        "Choose whether events shall be generated" annotation (Evaluate=true);
    equation
      //y = sign(u);
      y = if generateEvent then (if u > 0 then 1 elseif u < 0 then -1 else 0)
         else (if noEvent(u > 0) then 1 elseif noEvent(u < 0) then -1 else 0);
      annotation (
        defaultComponentName="sign1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-80},{0,-80}}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Text(
              extent={{-90,72},{-18,24}},
              textColor={192,192,192},
              textString="sign"),
            Line(points={{0,80},{80,80}}),
            Rectangle(
              extent={{-2,2},{2,-4}},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <strong>sign</strong> of the input <strong>u</strong>:
</p>
<blockquote><pre>
     1  <strong>if</strong> u &gt; 0
y =  0  <strong>if</strong> u == 0
    -1  <strong>if</strong> u &lt; 0
</pre></blockquote>
<p>
The Boolean parameter generateEvent decides whether Events are generated at zero crossing (Modelica specification before 3) or not.
</p>
</html>"));
    end Sign;

    block Sqrt "Output the square root of the input (input >= 0 required)"
      extends Interfaces.SISO;

    equation
      y = sqrt(u);
      annotation (
        defaultComponentName="sqrt1",
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{-90,-80},{68,-80}}, color={192,192,192}),
            Polygon(
              points={{90,-80},{68,-72},{68,-88},{90,-80}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-80,-80},{-79.2,-68.7},{-78.4,-64},{-76.8,-57.3},{-73.6,-47.9},
                  {-67.9,-36.1},{-59.1,-22.2},{-46.2,-6.49},{-28.5,10.7},{-4.42,
                  30},{27.7,51.3},{69.5,74.7},{80,80}},
              smooth=Smooth.Bezier),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-88},{-80,68}}, color={192,192,192}),
            Text(
              extent={{-8,-4},{64,-52}},
              textColor={192,192,192},
              textString="sqrt")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <em>square root</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>sqrt</strong>( u );
</pre></blockquote>
<p>
The input shall be zero or positive.
Otherwise an error occurs.
</p>

</html>"));
    end Sqrt;

    block Sin "Output the sine of the input"
      extends Interfaces.SISO(u(unit="rad"));
    equation
      y = Modelica.Math.sin(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-80},{-80,68}}, color={192,192,192}),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Line(
              points={{-80,0},{-68.7,34.2},{-61.5,53.1},{-55.1,66.4},{-49.4,74.6},
                  {-43.8,79.1},{-38.2,79.8},{-32.6,76.6},{-26.9,69.7},{-21.3,59.4},
                  {-14.9,44.1},{-6.83,21.2},{10.1,-30.8},{17.3,-50.2},{23.7,-64.2},
                  {29.3,-73.1},{35,-78.4},{40.6,-80},{46.2,-77.6},{51.9,-71.5},{
                  57.5,-61.9},{63.9,-47.2},{72,-24.8},{80,0}},
              smooth=Smooth.Bezier),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{12,84},{84,36}},
              textColor={192,192,192},
              textString="sin")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <strong>sine</strong> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>sin</strong>( u );
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/sin.png\"
     alt=\"sin.png\">
</p>

</html>"));
    end Sin;

    block Cos "Output the cosine of the input"
      extends Interfaces.SISO(u(unit="rad"));

    equation
      y = Modelica.Math.cos(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-80},{-80,68}}, color={192,192,192}),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-80,80},{-74.4,78.1},{-68.7,72.3},{-63.1,63},{-56.7,48.7},
                  {-48.6,26.6},{-29.3,-32.5},{-22.1,-51.7},{-15.7,-65.3},{-10.1,-73.8},
                  {-4.42,-78.8},{1.21,-79.9},{6.83,-77.1},{12.5,-70.6},{18.1,-60.6},
                  {24.5,-45.7},{32.6,-23},{50.3,31.3},{57.5,50.7},{63.9,64.6},{
                  69.5,73.4},{75.2,78.6},{80,80}},
              smooth=Smooth.Bezier),
            Text(
              extent={{-36,82},{36,34}},
              textColor={192,192,192},
              textString="cos")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <strong>cos</strong> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>cos</strong>( u );
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/cos.png\"
     alt=\"cos.png\">
</p>

</html>"));
    end Cos;

    block Tan "Output the tangent of the input"
      extends Interfaces.SISO(u(unit="rad"));

    equation
      y = Modelica.Math.tan(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Line(
              points={{-80,-80},{-78.4,-68.4},{-76.8,-59.7},{-74.4,-50},{-71.2,-40.9},
                  {-67.1,-33},{-60.7,-24.8},{-51.1,-17.2},{-35.8,-9.98},{-4.42,-1.07},
                  {33.4,9.12},{49.4,16.2},{59.1,23.2},{65.5,30.6},{70.4,39.1},{
                  73.6,47.4},{76,56.1},{77.6,63.8},{80,80}},
              smooth=Smooth.Bezier),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-90,72},{-18,24}},
              textColor={192,192,192},
              textString="tan")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong>
as <strong>tan</strong> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>tan</strong>( u );
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/tan.png\"
     alt=\"tan.png\">
</p>

</html>"));
    end Tan;

    block Asin "Output the arc sine of the input"
      extends Interfaces.SISO(y(unit="rad"));

    equation
      y = Modelica.Math.asin(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Line(
              points={{-80,-80},{-79.2,-72.8},{-77.6,-67.5},{-73.6,-59.4},{-66.3,
                  -49.8},{-53.5,-37.3},{-30.2,-19.7},{37.4,24.8},{57.5,40.8},{
                  68.7,52.7},{75.2,62.2},{77.6,67.5},{80,80}},
              smooth=Smooth.Bezier),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-88,78},{-16,30}},
              textColor={192,192,192},
              textString="asin")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>sine-inverse</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>asin</strong>( u );
</pre></blockquote>
<p>
The absolute value of the input <strong>u</strong> need to
be less or equal to one (<strong>abs</strong>( u ) <= 1).
Otherwise an error occurs.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/asin.png\"
     alt=\"atan.png\">
</p>

</html>"));
    end Asin;

    block Acos "Output the arc cosine of the input"
      extends Interfaces.SISO(y(unit="rad"));
    equation
      y = Modelica.Math.acos(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-80,80},{-79.2,72.8},{-77.6,67.5},{-73.6,59.4},{-66.3,49.8},
                  {-53.5,37.3},{-30.2,19.7},{37.4,-24.8},{57.5,-40.8},{68.7,-52.7},
                  {75.2,-62.2},{77.6,-67.5},{80,-80}},
              smooth=Smooth.Bezier),
            Line(points={{0,-88},{0,68}}, color={192,192,192}),
            Line(points={{-90,-80},{68,-80}}, color={192,192,192}),
            Polygon(
              points={{90,-80},{68,-72},{68,-88},{90,-80}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-86,-14},{-14,-62}},
              textColor={192,192,192},
              textString="acos")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>cosine-inverse</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>acos</strong>( u );
</pre></blockquote>
<p>
The absolute value of the input <strong>u</strong> need to
be less or equal to one (<strong>abs</strong>( u ) <= 1).
Otherwise an error occurs.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/acos.png\"
     alt=\"acos.png\">
</p>

</html>"));
    end Acos;

    block Atan "Output the arc tangent of the input"
      extends Interfaces.SISO(y(unit="rad"));
    equation
      y = Modelica.Math.atan(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Line(
              points={{-80,-80},{-52.7,-75.2},{-37.4,-69.7},{-26.9,-63},{-19.7,-55.2},
                  {-14.1,-45.8},{-10.1,-36.4},{-6.03,-23.9},{-1.21,-5.06},{5.23,
                  21},{9.25,34.1},{13.3,44.2},{18.1,52.9},{24.5,60.8},{33.4,67.6},
                  {47,73.6},{69.5,78.6},{80,80}},
              smooth=Smooth.Bezier),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-86,68},{-14,20}},
              textColor={192,192,192},
              textString="atan")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>tangent-inverse</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y= <strong>atan</strong>( u );
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/atan.png\"
     alt=\"atan.png\">
</p>

</html>"));
    end Atan;

    block Atan2 "Output atan(u1/u2) of the inputs u1 and u2"
      extends Interfaces.SI2SO(y(unit="rad"));
    equation
      y = Modelica.Math.atan2(u1, u2);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-80,-34.9},{-46.1,-31.4},{-29.4,-27.1},{-18.3,-21.5},{-10.3,
                  -14.5},{-2.03,-3.17},{7.97,11.6},{15.5,19.4},{24.3,25},{39,30},
                  {62.1,33.5},{80,34.9}},
              smooth=Smooth.Bezier),
            Line(
              points={{-80,45.1},{-45.9,48.7},{-29.1,52.9},{-18.1,58.6},{-10.2,
                  65.8},{-1.82,77.2},{0,80}},
              smooth=Smooth.Bezier),
            Line(
              points={{0,-80},{8.93,-67.2},{17.1,-59.3},{27.3,-53.6},{42.1,-49.4},
                  {69.9,-45.8},{80,-45.1}},
              smooth=Smooth.Bezier),
            Text(
              extent={{-90,-46},{-18,-94}},
              textColor={192,192,192},
              textString="atan2")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>tangent-inverse</em> of the input <strong>u1</strong> divided by
input <strong>u2</strong>:
</p>
<blockquote><pre>
y = <strong>atan2</strong>( u1, u2 );
</pre></blockquote>
<p>
u1 and u2 shall not be zero at the same time instant.
<strong>Atan2</strong> uses the sign of u1 and u2 in order to construct
the solution in the range -180 deg &le; y &le; 180 deg, whereas
block <strong>Atan</strong> gives a solution in the range
-90 deg &le; y &le; 90 deg.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/atan2.png\"
     alt=\"atan2.png\">
</p>

</html>"));
    end Atan2;

    block Sinh "Output the hyperbolic sine of the input"
      extends Interfaces.SISO;

    equation
      y = Modelica.Math.sinh(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-86,80},{-14,32}},
              textColor={192,192,192},
              textString="sinh"),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Line(
              points={{-80,-80},{-76,-65.4},{-71.2,-51.4},{-65.5,-38.8},{-59.1,-28.1},
                  {-51.1,-18.7},{-41.4,-11.4},{-27.7,-5.5},{-4.42,-0.653},{24.5,
                  4.57},{39,10.1},{49.4,17.2},{57.5,25.9},{63.9,35.8},{69.5,47.4},
                  {74.4,60.4},{78.4,73.8},{80,80}},
              smooth=Smooth.Bezier),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>hyperbolic sine</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>sinh</strong>( u );
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/sinh.png\"
     alt=\"sinh.png\">
</p>

</html>"));
    end Sinh;

    block Cosh "Output the hyperbolic cosine of the input"
      extends Interfaces.SISO;
    equation
      y = Modelica.Math.cosh(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Text(
              extent={{4,66},{66,20}},
              textColor={192,192,192},
              textString="cosh"),
            Line(
              points={{-80,80},{-77.6,61.1},{-74.4,39.3},{-71.2,20.7},{-67.1,1.29},
                  {-63.1,-14.6},{-58.3,-29.8},{-52.7,-43.5},{-46.2,-55.1},{-39,-64.3},
                  {-30.2,-71.7},{-18.9,-77.1},{-4.42,-79.9},{10.9,-79.1},{23.7,-75.2},
                  {34.2,-68.7},{42.2,-60.6},{48.6,-51.2},{54.3,-40},{59.1,-27.5},
                  {63.1,-14.6},{67.1,1.29},{71.2,20.7},{74.4,39.3},{77.6,61.1},{
                  80,80}},
              smooth=Smooth.Bezier),
            Line(points={{-90,-86.083},{68,-86.083}}, color={192,192,192}),
            Polygon(
              points={{90,-86.083},{68,-78.083},{68,-94.083},{90,-86.083}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>hyperbolic cosine</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>cosh</strong>( u );
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/cosh.png\"
     alt=\"cosh.png\">
</p>

</html>"));
    end Cosh;

    block Tanh "Output the hyperbolic tangent of the input"
      extends Interfaces.SISO;
    equation
      y = Modelica.Math.tanh(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{0,-90},{0,84}}, color={192,192,192}),
            Line(points={{-100,0},{84,0}}, color={192,192,192}),
            Line(
              points={{-80,-80},{-47.8,-78.7},{-35.8,-75.7},{-27.7,-70.6},{-22.1,
                  -64.2},{-17.3,-55.9},{-12.5,-44.3},{-7.64,-29.2},{-1.21,-4.82},
                  {6.83,26.3},{11.7,42},{16.5,54.2},{21.3,63.1},{26.9,69.9},{34.2,
                  75},{45.4,78.4},{72,79.9},{80,80}},
              smooth=Smooth.Bezier),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-88,72},{-16,24}},
              textColor={192,192,192},
              textString="tanh"),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>hyperbolic tangent</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>tanh</strong>( u );
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/tanh.png\"
     alt=\"tanh.png\">
</p>

</html>"));
    end Tanh;

    block Exp "Output the exponential (base e) of the input"
      extends Interfaces.SISO;

    equation
      y = Modelica.Math.exp(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-86,50},{-14,2}},
              textColor={192,192,192},
              textString="exp"),
            Line(points={{-80,-80},{-31,-77.9},{-6.03,-74},{10.9,-68.4},{23.7,-61},
                  {34.2,-51.6},{43,-40.3},{50.3,-27.8},{56.7,-13.5},{62.3,2.23},{
                  67.1,18.6},{72,38.2},{76,57.6},{80,80}}),
            Line(
              points={{-90,-80.3976},{68,-80.3976}},
              color={192,192,192},
              smooth=Smooth.Bezier),
            Polygon(
              points={{90,-80.3976},{68,-72.3976},{68,-88.3976},{90,-80.3976}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>exponential</em> (of base e) of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>exp</strong>( u );
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/exp.png\"
     alt=\"exp.png\">
</p>

</html>"));
    end Exp;

    block Power "Output the power to a base of the input"
      extends Interfaces.SISO;
      parameter Real base = Modelica.Constants.e "Base of power" annotation(Evaluate=true);
      parameter Boolean useExp = true "Use exp function in implementation" annotation(Evaluate=true);
    equation
      y = if useExp then Modelica.Math.exp(u*Modelica.Math.log(base)) else base ^ u;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-86,50},{-14,2}},
              textColor={192,192,192},
              textString="^"),
            Line(points={{-80,-80},{-31,-77.9},{-6.03,-74},{10.9,-68.4},{23.7,-61},
                  {34.2,-51.6},{43,-40.3},{50.3,-27.8},{56.7,-13.5},{62.3,2.23},{
                  67.1,18.6},{72,38.2},{76,57.6},{80,80}}),
            Line(
              points={{-90,-80.3976},{68,-80.3976}},
              color={192,192,192},
              smooth=Smooth.Bezier),
            Polygon(
              points={{90,-80.3976},{68,-72.3976},{68,-88.3976},{90,-80.3976}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
power to the parameter <em>base</em> of the input <strong>u</strong>.
If the boolean parameter <strong>useExp</strong> is true, the output is determined by:
</p>
<blockquote><pre>
y = <strong>exp</strong> ( u * <strong>log</strong> (base) )
</pre></blockquote>
<p>
otherwise:
</p>
<blockquote><pre>
y = base <strong>^</strong> u;
</pre></blockquote>

</html>"));
    end Power;

    block Log
      "Output the logarithm (default base e) of the input (input > 0 required)"

      extends Interfaces.SISO;
      parameter Real base = Modelica.Constants.e "Base of logarithm" annotation(Evaluate=true);

    equation
      y = Modelica.Math.log(u)/Modelica.Math.log(base);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{-80,-80},{-80,68}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-80,-80},{-79.2,-50.6},{-78.4,-37},{-77.6,-28},{-76.8,-21.3},
                  {-75.2,-11.4},{-72.8,-1.31},{-69.5,8.08},{-64.7,17.9},{-57.5,28},
                  {-47,38.1},{-31.8,48.1},{-10.1,58},{22.1,68},{68.7,78.1},{80,80}},
              smooth=Smooth.Bezier),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-6,-24},{66,-72}},
              textColor={192,192,192},
              textString="log")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>logarithm</em> to the parameter <em>base</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>log</strong>( u ) / <strong>log</strong>( base );
</pre></blockquote>
<p>
An error occurs if the input <strong>u</strong> is
zero or negative.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/log.png\"
     alt=\"log.png\">
</p>

</html>"));
    end Log;

    block Log10 "Output the base 10 logarithm of the input (input > 0 required)"

      extends Interfaces.SISO;
    equation
      y = Modelica.Math.log10(u);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Line(
              points={{-79.8,-80},{-79.2,-50.6},{-78.4,-37},{-77.6,-28},{-76.8,-21.3},
                  {-75.2,-11.4},{-72.8,-1.31},{-69.5,8.08},{-64.7,17.9},{-57.5,28},
                  {-47,38.1},{-31.8,48.1},{-10.1,58},{22.1,68},{68.7,78.1},{80,80}},
              smooth=Smooth.Bezier),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-80},{-80,68}}, color={192,192,192}),
            Text(
              extent={{-30,-22},{60,-70}},
              textColor={192,192,192},
              textString="log10")}),
        Documentation(info="<html>
<p>
This blocks computes the output <strong>y</strong> as the
<em>base 10 logarithm</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>log10</strong>( u );
</pre></blockquote>
<p>
An error occurs if the input <strong>u</strong> is
zero or negative.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Math/log10.png\"
     alt=\"log10.png\">
</p>

</html>"));
    end Log10;

    block WrapAngle "Wrap angle to interval ]-pi,pi] or [0,2*pi["

      extends Interfaces.SISO(u(unit="rad"), y(unit="rad"));
      parameter Boolean positiveRange=false "Use only positive output range, if true";

    equation
      y = Modelica.Math.wrapAngle(u,positiveRange);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{-80,54},{-80,54},{-60,80},{-60,-80},{60,80},{60,-80},{
                  80,-52}}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This blocks wraps the input angle into the interval ]-pi,pi], if <code>positiveRange == false</code>.
Otherwise the input angle <code>u</code> is wrapped to the interval [0,2*pi[.
</p>

</html>"));
    end WrapAngle;

    block RealToInteger "Convert Real to Integer signal"
      extends Blocks.Icons.IntegerBlock;
    public
      Interfaces.RealInput u "Connector of Real input signal" annotation (
          Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.IntegerOutput y "Connector of Integer output signal" annotation (
         Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      y = if (u > 0) then integer(floor(u + 0.5)) else integer(ceil(u - 0.5));
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
            Text(
              textColor={0,0,127},
              extent={{-100.0,-40.0},{0.0,40.0}},
              textString="R"),
            Text(
              textColor={255,127,0},
              extent={{20.0,-40.0},{120.0,40.0}},
              textString="I"),
            Polygon(
              lineColor={255,127,0},
              fillColor={255,127,0},
              fillPattern=FillPattern.Solid,
              points={{50.0,0.0},{30.0,20.0},{30.0,10.0},{0.0,10.0},{0.0,-10.0},{
                  30.0,-10.0},{30.0,-20.0},{50.0,0.0}})}), Documentation(info="<html>
<p>
This block computes the output <strong>y</strong>
as <em>nearest integer value</em> of the input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>integer</strong>( <strong>floor</strong>( u + 0.5 ) )  for  u &gt; 0;
y = <strong>integer</strong>( <strong>ceil </strong>( u - 0.5 ) )  for  u &lt; 0;
</pre></blockquote>
</html>"));
    end RealToInteger;

    block IntegerToReal "Convert Integer to Real signals"
      extends Blocks.Icons.Block;
      Interfaces.IntegerInput u "Connector of Integer input signal" annotation (
          Placement(transformation(extent={{-140,-20},{-100,20}})));
      Interfaces.RealOutput y "Connector of Real output signal" annotation (
          Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      y = u;
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
            Text(
              textColor={255,127,0},
              extent={{-120.0,-40.0},{-20.0,40.0}},
              textString="I"),
            Text(
              textColor={0,0,127},
              extent={{0.0,-40.0},{100.0,40.0}},
              textString="R"),
            Polygon(
              lineColor={0,0,127},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              points={{10.0,0.0},{-10.0,20.0},{-10.0,10.0},{-40.0,10.0},{-40.0,-10.0},
                  {-10.0,-10.0},{-10.0,-20.0},{10.0,0.0}})}), Documentation(info="<html>
<p>
This block computes the output <strong>y</strong>
as <em>Real equivalent</em> of the Integer input <strong>u</strong>:
</p>
<blockquote><pre>
y = u;
</pre></blockquote>
<p>where <strong>u</strong> is of Integer and <strong>y</strong> of Real type.</p>
</html>"));
    end IntegerToReal;

    block BooleanToReal "Convert Boolean to Real signal"
      extends Interfaces.partialBooleanSI;
      parameter Real realTrue=1.0 "Output signal for true Boolean input";
      parameter Real realFalse=0.0 "Output signal for false Boolean input";

      Blocks.Interfaces.RealOutput y "Connector of Real output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      y = if u then realTrue else realFalse;
      annotation (Documentation(info="<html>
<p>
This block computes the output <strong>y</strong>
as <em>Real equivalent</em> of the Boolean input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>if</strong> u <strong>then</strong> realTrue <strong>else</strong> realFalse;
</pre></blockquote>
<p>where <strong>u</strong> is of Boolean and <strong>y</strong> of Real type,
and <strong>realTrue</strong> and <strong>realFalse</strong> are parameters.
</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Text(
              extent={{-86,92},{-6,10}},
              textColor={255,0,255},
              textString="B"),
            Polygon(
              points={{-12,-46},{-32,-26},{-32,-36},{-64,-36},{-64,-56},{-32,-56},
                  {-32,-66},{-12,-46}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Text(
              extent={{8,-4},{92,-94}},
              textString="R",
              textColor={0,0,127})}));
    end BooleanToReal;

    block BooleanToInteger "Convert Boolean to Integer signal"
      extends Interfaces.partialBooleanSI;
      parameter Integer integerTrue=1 "Output signal for true Boolean input";
      parameter Integer integerFalse=0 "Output signal for false Boolean input";

      Blocks.Interfaces.IntegerOutput y "Connector of Integer output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      y = if u then integerTrue else integerFalse;
      annotation (Documentation(info="<html>
<p>
This block computes the output <strong>y</strong>
as <em>Integer equivalent</em> of the Boolean input <strong>u</strong>:
</p>
<blockquote><pre>
y = <strong>if</strong> u <strong>then</strong> integerTrue <strong>else</strong> integerFalse;
</pre></blockquote>
<p>where <strong>u</strong> is of Boolean and <strong>y</strong> of Integer type,
and <strong>integerTrue</strong> and <strong>integerFalse</strong> are parameters.
</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Text(
              extent={{-86,92},{-6,10}},
              textColor={255,0,255},
              textString="B"),
            Polygon(
              points={{-12,-46},{-32,-26},{-32,-36},{-64,-36},{-64,-56},{-32,-56},
                  {-32,-66},{-12,-46}},
              lineColor={255,170,85},
              fillColor={255,170,85},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{8,-4},{92,-94}},
              textColor={255,170,85},
              textString="I")}));
    end BooleanToInteger;

    block RealToBoolean "Convert Real to Boolean signal"

      Blocks.Interfaces.RealInput u "Connector of Real input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      extends Interfaces.partialBooleanSO;
      parameter Real threshold=0.5
        "Output signal y is true, if input u >= threshold";

    equation
      y = u >= threshold;
      annotation (Documentation(info="<html>
<p>
This block computes the Boolean output <strong>y</strong>
from the Real input <strong>u</strong> by the equation:
</p>

<blockquote><pre>
y = u &ge; threshold;
</pre></blockquote>

<p>
where <strong>threshold</strong> is a parameter.
</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Text(
              extent={{-86,92},{-6,10}},
              textColor={0,0,127},
              textString="R"),
            Polygon(
              points={{-12,-46},{-32,-26},{-32,-36},{-64,-36},{-64,-56},{-32,-56},
                  {-32,-66},{-12,-46}},
              lineColor={255,0,255},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{8,-4},{92,-94}},
              textColor={255,0,255},
              textString="B")}));
    end RealToBoolean;

    block IntegerToBoolean "Convert Integer to Boolean signal"

      Blocks.Interfaces.IntegerInput u "Connector of Integer input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      extends Interfaces.partialBooleanSO;
      parameter Integer threshold=1
        "Output signal y is true, if input u >= threshold";

    equation
      y = u >= threshold;
      annotation (Documentation(info="<html>
<p>
This block computes the Boolean output <strong>y</strong>
from the Integer input <strong>u</strong> by the equation:
</p>

<blockquote><pre>
y = u &ge; threshold;
</pre></blockquote>

<p>
where <strong>threshold</strong> is a parameter.
</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={
            Text(
              extent={{-86,92},{-6,10}},
              textColor={255,128,0},
              textString="I"),
            Polygon(
              points={{-12,-46},{-32,-26},{-32,-36},{-64,-36},{-64,-56},{-32,-56},
                  {-32,-66},{-12,-46}},
              lineColor={255,0,255},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{8,-4},{92,-94}},
              textColor={255,0,255},
              textString="B")}));
    end IntegerToBoolean;

    block RectangularToPolar
      "Convert rectangular coordinates to polar coordinates"
      extends Blocks.Icons.Block;
      Blocks.Interfaces.RealInput u_re
        "Real part of rectangular representation"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Blocks.Interfaces.RealInput u_im
        "Imaginary part of rectangular representation"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Blocks.Interfaces.RealOutput y_abs "Length of polar representation"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
      Blocks.Interfaces.RealOutput y_arg(unit="rad")
        "Angle of polar representation"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));

    equation
      y_abs = sqrt(u_re*u_re + u_im*u_im);
      y_arg = Modelica.Math.atan2(u_im, u_re);
      annotation (Icon(graphics={
            Text(
              extent={{-90,80},{-20,40}},
              textString="re"),
            Text(
              extent={{-90,-40},{-20,-80}},
              textString="im"),
            Text(
              extent={{20,80},{90,40}},
              textString="abs"),
            Text(
              extent={{20,-40},{90,-80}},
              textString="arg")}), Documentation(info="<html>
<p>
The input values of this block are the rectangular components
<code>u_re</code> and <code>u_im</code> of a phasor in two dimensions.
This block calculates the length <code>y_abs</code> and
the angle <code>y_arg</code> of the polar representation of this phasor.
</p>

<blockquote><pre>
y_abs = abs(u_re + j*u_im) = sqrt( u_re<sup>2</sup> + u_im<sup>2</sup> )
y_arg = arg(u_re + j*u_im) = atan2(u_im, u_re)
</pre></blockquote>
</html>"));
    end RectangularToPolar;

    block PolarToRectangular
      "Convert polar coordinates to rectangular coordinates"
      extends Blocks.Icons.Block;
      Blocks.Interfaces.RealInput u_abs "Length of polar representation"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Blocks.Interfaces.RealInput u_arg(unit="rad")
        "Angle of polar representation"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Blocks.Interfaces.RealOutput y_re
        "Real part of rectangular representation"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
      Blocks.Interfaces.RealOutput y_im
        "Imaginary part of rectangular representation"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));

    equation
      y_re = u_abs*Modelica.Math.cos(u_arg);
      y_im = u_abs*Modelica.Math.sin(u_arg);
      annotation (Icon(graphics={
            Text(
              extent={{-90,80},{-20,40}},
              textString="abs"),
            Text(
              extent={{-90,-40},{-20,-80}},
              textString="arg"),
            Text(
              extent={{20,80},{90,40}},
              textString="re"),
            Text(
              extent={{20,-40},{90,-80}},
              textString="im")}), Documentation(info="<html>
<p>
The input values of this block are the polar components <code>uabs</code> and <code>uarg</code> of a phasor.
This block calculates the components <code>y_re</code> and <code>y_im</code> of the rectangular representation of this phasor.
</p>
<blockquote><pre>
y_re = u_abs * cos( u_arg )
y_im = u_abs * sin( u_arg )
</pre></blockquote>
</html>"));
    end PolarToRectangular;

    block Mean "Calculate mean over period 1/f"
      extends Blocks.Interfaces.SISO;
      parameter SI.Frequency f(start=50) "Base frequency";
      parameter Real x0=0 "Start value of integrator state";
      parameter Boolean yGreaterOrEqualZero=false
        "= true, if output y is guaranteed to be >= 0 for the exact solution"
        annotation (Evaluate=true, Dialog(tab="Advanced"));
    protected
      parameter SI.Time t0(fixed=false) "Start time of simulation";
      Real x "Integrator state";
      discrete Real y_last "Last sampled mean value";
    initial equation
      t0 = time;
      x = x0;
      y_last = 0;
    equation
      der(x) = u;
      when sample(t0 + 1/f, 1/f) then
        y_last = if not yGreaterOrEqualZero then f*pre(x) else max(0.0, f*pre(x));
        reinit(x, 0);
      end when;
      y = y_last;
      annotation (Documentation(info="<html>
<p>
This block calculates the mean of the input signal u over the given period 1/f:
</p>
<blockquote><pre>
1 T
- &int; u(t) dt
T 0
</pre></blockquote>
<p>
Note: The output is updated after each period defined by 1/f.
</p>

<p>
If parameter <strong>yGreaterOrEqualZero</strong> in the Advanced tab is <strong>true</strong> (default = <strong>false</strong>),
then the modeller provides the information that the mean of the input signal is guaranteed
to be &ge; 0 for the exact solution. However, due to inaccuracies in the numerical integration scheme,
the output might be slightly negative. If this parameter is set to true, then the output is
explicitly set to 0.0, if the mean value results in a negative value.
</p>
</html>"),   Icon(graphics={Text(
              extent={{-80,60},{80,20}},
              textString="mean"), Text(
              extent={{-80,-20},{80,-60}},
              textString="f=%f")}));
    end Mean;

    block RectifiedMean "Calculate rectified mean over period 1/f"
      extends Blocks.Interfaces.SISO;
      parameter SI.Frequency f(start=50) "Base frequency";
      parameter Real x0=0 "Start value of integrator state";
      Mean mean(final f=f, final x0=x0)
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
      Blocks.Math.Abs abs1
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
    equation
      connect(u, abs1.u) annotation (Line(
          points={{-120,0},{-62,0}}, color={0,0,127}));
      connect(abs1.y, mean.u) annotation (Line(
          points={{-39,0},{-2,0}}, color={0,0,127}));
      connect(mean.y, y) annotation (Line(
          points={{21,0},{110,0}}, color={0,0,127}));
      annotation (Documentation(info="<html>
<p>
This block calculates the rectified mean of the input signal u over the given period 1/f, using the
<a href=\"modelica://Modelica.Blocks.Math.Mean\">mean block</a>.
</p>
<p>
Note: The output is updated after each period defined by 1/f.
</p>
</html>"),   Icon(graphics={Text(
              extent={{-80,60},{80,20}},
              textString="RM"), Text(
              extent={{-80,-20},{80,-60}},
              textString="f=%f")}));
    end RectifiedMean;

    block ContinuousMean
      "Calculates the empirical expectation (mean) value of its input signal"
      extends Blocks.Icons.Block;
      parameter SI.Time t_eps(min= 100*Modelica.Constants.eps)=1e-7
        "Mean value calculation starts at startTime + t_eps"
        annotation(Dialog(group="Advanced"));

      Blocks.Interfaces.RealInput u "Noisy input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y
        "Expectation (mean) value of the input signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    protected
      Real mu "Internal integrator variable";
      parameter Real t_0(fixed=false) "Start time";
    initial equation
      t_0 = time;
      mu  = u;
    equation
      der(mu) = noEvent(if time >= t_0 + t_eps then (u-mu)/(time-t_0) else 0);
      y       = noEvent(if time >= t_0 + t_eps then mu                else u);

      annotation (Documentation(revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>",                                     info="<html>
<p>This block continuously calculates the mean value of its input signal. It uses the function:</p>
<blockquote><pre>
     integral( u over time)
y = ------------------------
        time - startTime
</pre></blockquote>
<p>This can be used to determine the empirical expectation value of a random signal, such as generated by the <a href=\"modelica://Modelica.Blocks.Noise\">Noise</a> blocks.</p>
<p>The parameter t_eps is used to guard against division by zero (the mean value computation
starts at &lt;<em>simulation start time</em>&gt; + t_eps and before that time instant y = u).</p>
<p>See also the <a href=\"modelica://Modelica.Blocks.Math.Mean\">Mean</a> block for a sampled implementation.</p>

<p>
This block is demonstrated in the examples
<a href=\"modelica://Modelica.Blocks.Examples.Noise.UniformNoiseProperties\">UniformNoiseProperties</a> and
<a href=\"modelica://Modelica.Blocks.Examples.Noise.NormalNoiseProperties\">NormalNoiseProperties</a>.
</p>
</html>"),   Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics={
            Polygon(
              points={{94,0},{72,8},{72,-8},{94,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-86,0},{72,0}}, color={192,192,192}),
            Line(points={{-76,68},{-76,-80}}, color={192,192,192}),
            Polygon(
              points={{-76,90},{-84,68},{-68,68},{-76,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
               points={{-76,-31},{-62,-31},{-62,-15},{-54,-15},{-54,-63},{-46,-63},
                  {-46,-41},{-38,-41},{-38,43},{-30,43},{-30,11},{-30,11},{-30,-49},
                  {-20,-49},{-20,-31},{-10,-31},{-10,-59},{0,-59},{0,23},{6,23},{6,
                  37},{12,37},{12,-19},{22,-19},{22,-7},{28,-7},{28,-37},{38,-37},
                  {38,35},{48,35},{48,1},{56,1},{56,-65},{66,-65}},
                color={215,215,215}),
            Line(
              points={{-76,-24},{70,-24}})}));
    end ContinuousMean;

    block RootMeanSquare "Calculate root mean square over period 1/f"
      extends Blocks.Interfaces.SISO;
      parameter SI.Frequency f(start=50) "Base frequency";
      parameter Real x0=0 "Start value of integrator state";
      MultiProduct product(nu=2)
        annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
      Mean mean(
        final f=f,
        final yGreaterOrEqualZero=true,
        final x0=x0)
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
      Blocks.Math.Sqrt sqrt1
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation

      connect(product.y, mean.u) annotation (Line(
          points={{-18.3,0},{-2,0}}, color={0,0,127}));
      connect(mean.y, sqrt1.u) annotation (Line(
          points={{21,0},{38,0}}, color={0,0,127}));
      connect(sqrt1.y, y) annotation (Line(
          points={{61,0},{110,0}}, color={0,0,127}));
      connect(u, product.u[1]) annotation (Line(
          points={{-120,0},{-60,0},{-60,3.5},{-40,3.5}}, color={0,0,127}));
      connect(u, product.u[2]) annotation (Line(
          points={{-120,0},{-60,0},{-60,-3.5},{-40,-3.5}}, color={0,0,127}));
      annotation (Documentation(info="<html>
<p>
This block calculates the root mean square of the input signal u over the given period 1/f, using the
<a href=\"modelica://Modelica.Blocks.Math.Mean\">mean block</a>.
</p>
<p>
Note: The output is updated after each period defined by 1/f.
</p>
</html>"),   Icon(graphics={Text(
              extent={{-80,60},{80,20}},
              textString="RMS"), Text(
              extent={{-80,-20},{80,-60}},
              textString="f=%f")}));
    end RootMeanSquare;

    block Variance "Calculates the empirical variance of its input signal"
      extends Blocks.Icons.Block;
      parameter SI.Time t_eps(min=100*Modelica.Constants.eps)=1e-7
        "Variance calculation starts at startTime + t_eps"
        annotation(Dialog(group="Advanced"));

      Blocks.Interfaces.RealInput u "Noisy input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y "Variance of the input signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    protected
      Real mu "Mean value (state variable)";
      Real var "Variance (state variable)";
      parameter Real t_0(fixed=false) "Start time";
    initial equation
      t_0 = time;
      mu  = u;
      var = 0;
    equation
      der(mu)  = noEvent(if time >= t_0 + t_eps then (u-mu)/(time-t_0)             else 0);
      der(var) = noEvent(if time >= t_0 + t_eps then ((u-mu)^2 - var)/(time - t_0) else 0);
      y        = noEvent(if time >= t_0 + t_eps then max(var,0)                    else 0);

      annotation (Documentation(revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>",   info="<html>
<p>
This block calculates the empirical variance of its input signal. It is based on the formula
(but implemented in a more reliable numerical way):
</p>
<blockquote><pre>
y = mean(  (u - mean(u))^2  )
</pre></blockquote>

<p>The parameter t_eps is used to guard against division by zero (the variance computation
starts at &lt;<em>simulation start time</em>&gt; + t_eps and before that time instant y = 0).</p>
<p>The variance of a signal is also equal to its mean power.</p>

<p>
This block is demonstrated in the examples
<a href=\"modelica://Modelica.Blocks.Examples.Noise.UniformNoiseProperties\">UniformNoiseProperties</a> and
<a href=\"modelica://Modelica.Blocks.Examples.Noise.NormalNoiseProperties\">NormalNoiseProperties</a>.
</p>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{-76,68},{-76,-80}}, color={192,192,192}),
            Line(points={{-86,0},{72,0}}, color={192,192,192}),
            Line(
               points={{-76,-13},{-62,-13},{-62,3},{-54,3},{-54,-45},{-46,-45},{-46,
                  -23},{-38,-23},{-38,61},{-30,61},{-30,29},{-30,29},{-30,-31},{-20,
                  -31},{-20,-13},{-10,-13},{-10,-41},{0,-41},{0,41},{6,41},{6,55},
                  {12,55},{12,-1},{22,-1},{22,11},{28,11},{28,-19},{38,-19},{38,53},
                  {48,53},{48,19},{56,19},{56,-47},{66,-47}},
                color={215,215,215}),
            Polygon(
              points={{94,0},{72,8},{72,-8},{94,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-76,66},{70,66}},
              color={215,215,215}),
            Line(
              points={{-16,0},{-16,48}}),
            Polygon(
              points={{-76,90},{-84,68},{-68,68},{-76,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-16,66},{-24,44},{-8,44},{-16,66}},
              fillPattern=FillPattern.Solid)}));
    end Variance;

    block StandardDeviation
      "Calculates the empirical standard deviation of its input signal"
      extends Blocks.Icons.Block;
      parameter SI.Time t_eps(min=100*Modelica.Constants.eps)=1e-7
        "Standard deviation calculation starts at startTime + t_eps"
        annotation(Dialog(group="Advanced"));

      Blocks.Interfaces.RealInput u "Noisy input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y "Standard deviation of the input signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

      Blocks.Math.Variance variance(t_eps=t_eps)
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Blocks.Math.Sqrt sqrt1
        annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
    equation
      connect(variance.u, u) annotation (Line(
          points={{-62,0},{-120,0}}, color={0,0,127}));
      connect(sqrt1.u, variance.y) annotation (Line(
          points={{-22,0},{-39,0}}, color={0,0,127}));
      connect(sqrt1.y, y) annotation (Line(
          points={{1,0},{110,0}}, color={0,0,127}));
      annotation (Documentation(revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>",   info="<html>
<p>This block calculates the standard deviation of its input signal. The standard deviation is the square root of the signal&#39;s variance:</p>
<blockquote><pre>
y = sqrt( variance(u) )
</pre></blockquote>
<p>
The <a href=\"modelica://Modelica.Blocks.Math.Variance\">Variance</a> block is used to
calculate variance(u).
</p>
<p>The parameter t_eps is used to guard against division by zero (the computation of the standard deviation
starts at &lt;<em>simulation start time</em>&gt; + t_eps and before that time instant y = 0).
</p>

<p>
This block is demonstrated in the examples
<a href=\"modelica://Modelica.Blocks.Examples.Noise.UniformNoiseProperties\">UniformNoiseProperties</a> and
<a href=\"modelica://Modelica.Blocks.Examples.Noise.NormalNoiseProperties\">NormalNoiseProperties</a>.
</p>
</html>"),
        Icon(graphics={
            Line(points={{-76,68},{-76,-80}}, color={192,192,192}),
            Line(points={{-86,0},{72,0}}, color={192,192,192}),
            Line(
               points={{-76,-13},{-62,-13},{-62,3},{-54,3},{-54,-45},{-46,-45},{-46,
                  -23},{-38,-23},{-38,61},{-30,61},{-30,29},{-30,29},{-30,-31},{-20,
                  -31},{-20,-13},{-10,-13},{-10,-41},{0,-41},{0,41},{6,41},{6,55},
                  {12,55},{12,-1},{22,-1},{22,11},{28,11},{28,-19},{38,-19},{38,53},
                  {48,53},{48,19},{56,19},{56,-47},{66,-47}},
                color={215,215,215}),
            Polygon(
              points={{94,0},{72,8},{72,-8},{94,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-76,46},{70,46}},
              color={215,215,215}),
            Line(
              points={{-16,0},{-16,30}}),
            Polygon(
              points={{-76,90},{-84,68},{-68,68},{-76,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-16,46},{-24,24},{-8,24},{-16,46}},
              fillPattern=FillPattern.Solid)}));
    end StandardDeviation;

    block Harmonic "Calculate harmonic over period 1/f"
      extends Blocks.Icons.Block;
      parameter SI.Frequency f(start=50) "Base frequency";
      parameter Integer k(start=1) "Order of harmonic";
      parameter Boolean useConjugateComplex=false
        "Gives conjugate complex result if true"
        annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
      parameter Real x0Cos=0 "Start value of cos integrator state";
      parameter Real x0Sin=0 "Start value of sin integrator state";
      Sources.Cosine      sin1(
        final amplitude=sqrt(2),
        final f=k*f,
        final phase=0) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-80,70})));
      Blocks.Sources.Sine sin2(
        final amplitude=sqrt(2),
        final phase=0,
        final f=k*f) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-80,-70})));
      MultiProduct product1(nu=2)
        annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
      MultiProduct product2(nu=2)
        annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
      Mean mean1(final f=f, final x0=x0Cos)
        annotation (Placement(transformation(extent={{-20,30},{0,50}})));
      Mean mean2(final f=f, final x0=x0Sin)
        annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
      Blocks.Interfaces.RealInput u
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y_rms
        "Root mean square of polar representation"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
      Blocks.Interfaces.RealOutput y_arg(unit="rad")
        "Angle of polar representation"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
      Blocks.Math.RectangularToPolar rectangularToPolar
        annotation (Placement(transformation(extent={{40,-12},{60,8}})));
      Gain gain(final k=if useConjugateComplex then -1 else 1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={80,-30})));
    equation

      connect(product2.y, mean2.u) annotation (Line(
          points={{-38.3,-40},{-22,-40}}, color={0,0,127}));
      connect(product1.y, mean1.u) annotation (Line(
          points={{-38.3,40},{-22,40}}, color={0,0,127}));
      connect(mean1.y, rectangularToPolar.u_re) annotation (Line(
          points={{1,40},{20,40},{20,4},{38,4}}, color={0,0,127}));
      connect(mean2.y, rectangularToPolar.u_im) annotation (Line(
          points={{1,-40},{20,-40},{20,-8},{38,-8}}, color={0,0,127}));
      connect(rectangularToPolar.y_abs, y_rms) annotation (Line(
          points={{61,4},{80,4},{80,60},{110,60}}, color={0,0,127}));
      connect(sin1.y, product1.u[1]) annotation (Line(
          points={{-80,59},{-80,59},{-80,43.5},{-60,43.5}}, color={0,0,127}));
      connect(u, product1.u[2]) annotation (Line(
          points={{-120,0},{-80,0},{-80,36.5},{-60,36.5}}, color={0,0,127}));
      connect(u, product2.u[1]) annotation (Line(
          points={{-120,0},{-80,0},{-80,-36.5},{-60,-36.5}}, color={0,0,127}));
      connect(sin2.y, product2.u[2]) annotation (Line(
          points={{-80,-59},{-80,-43.5},{-60,-43.5}}, color={0,0,127}));
      connect(rectangularToPolar.y_arg, gain.u)
        annotation (Line(points={{61,-8},{80,-8},{80,-18}}, color={0,0,127}));
      connect(gain.y, y_arg)
        annotation (Line(points={{80,-41},{80,-60},{110,-60}}, color={0,0,127}));
      annotation (Documentation(info="<html>
<p>
This block calculates the root mean square and the phase angle of a single harmonic <em>k</em> of the input signal u over the given period 1/f, using the
<a href=\"modelica://Modelica.Blocks.Math.Mean\">mean block</a>.
</p>
<p>
Note: The output is updated after each period defined by 1/f.
</p>
<p>
Note:<br>
The harmonic is defined by <code>&radic;2 rms cos(k 2 &pi; f t - arg)</code> if useConjugateComplex=false (default)<br>
The harmonic is defined by <code>&radic;2 rms cos(k 2 &pi; f t + arg)</code> if useConjugateComplex=true
</p>
</html>"),   Icon(graphics={
            Text(
              extent={{-80,60},{80,20}},
              textString="H%k"),
            Text(
              extent={{-80,-20},{80,-60}},
              textString="f=%f"),
            Text(
              extent={{20,100},{100,60}},
              textString="rms"),
            Text(
              extent={{20,-60},{100,-100}},
              textString="arg")}));
    end Harmonic;

    block TotalHarmonicDistortion "Output the total harmonic distortion (THD)"
      extends Interfaces.SISO;
      parameter SI.Frequency f(start=1) "Base frequency";
      parameter Boolean useFirstHarmonic = true "THD with respect to first harmonic, if true; otherwise with respect to total RMS";

      Harmonic harmonic(
        final f=f,
        final k=1,
        final x0Cos=0,
        final x0Sin=0) annotation (Placement(transformation(extent={{-70,-62},{-50,-42}})));
      RootMeanSquare rootMeanSquare(final f=f, final x0=0) annotation (Placement(transformation(extent={{-70,20},{-50,40}})));
      Logical.GreaterThreshold greaterThreshold annotation (Placement(transformation(extent={{10,-70},{30,-50}})));
      Interfaces.BooleanOutput valid "True, if output y is valid" annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
      Division division annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      Nonlinear.Limiter limiter(uMin=Modelica.Constants.eps, uMax=Modelica.Constants.inf) annotation (Placement(transformation(extent={{10,-30},{30,-10}})));
      Pythagoras pythagoras(u1IsHypotenuse=true) annotation (Placement(transformation(extent={{10,0},{30,20}})));
      Logical.And andValid annotation (Placement(transformation(extent={{60,-50},{80,-30}})));
      Sources.BooleanExpression booleanExpression(final y=not useFirstHarmonic) annotation (Placement(transformation(extent={{-70,-30},{-50,-10}})));
      Logical.Switch switch1 annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
    equation
      connect(u, rootMeanSquare.u) annotation (Line(points={{-120,0},{-90,0},{-90,30},{-72,30}}, color={0,0,127}));
      connect(u, harmonic.u) annotation (Line(points={{-120,0},{-90,0},{-90,-52},{-72,-52}}, color={0,0,127}));
      connect(harmonic.y_rms, greaterThreshold.u) annotation (Line(points={{-49,-46},{-40,-46},{-40,-60},{8,-60}},  color={0,0,127}));
      connect(division.y, y) annotation (Line(points={{81,0},{110,0}}, color={0,0,127}));
      connect(pythagoras.u1, rootMeanSquare.y) annotation (Line(points={{8,16},{-30,16},{-30,30},{-49,30}},  color={0,0,127}));
      connect(pythagoras.y, division.u1) annotation (Line(points={{31,10},{50,10},{50,6},{58,6}}, color={0,0,127}));
      connect(pythagoras.valid, andValid.u1) annotation (Line(points={{31,4},{40,4},{40,-40},{58,-40}}, color={255,0,255}));
      connect(greaterThreshold.y, andValid.u2) annotation (Line(points={{31,-60},{40,-60},{40,-48},{58,-48}}, color={255,0,255}));
      connect(andValid.y, valid) annotation (Line(points={{81,-40},{90,-40},{90,-60},{110,-60}}, color={255,0,255}));
      connect(limiter.y, division.u2) annotation (Line(points={{31,-20},{50,-20},{50,-6},{58,-6}}, color={0,0,127}));
      connect(harmonic.y_rms, pythagoras.u2) annotation (Line(points={{-49,-46},{-40,-46},{-40,4},{8,4}},  color={0,0,127}));
      connect(switch1.u1, rootMeanSquare.y) annotation (Line(points={{-22,-12},{-30,-12},{-30,30},{-49,30}}, color={0,0,127}));
      connect(harmonic.y_rms, switch1.u3) annotation (Line(points={{-49,-46},{-40,-46},{-40,-28},{-22,-28}}, color={0,0,127}));
      connect(booleanExpression.y, switch1.u2) annotation (Line(points={{-49,-20},{-22,-20}}, color={255,0,255}));
      connect(switch1.y, limiter.u) annotation (Line(points={{1,-20},{8,-20}}, color={0,0,127}));
      annotation (defaultComponentName="thd",
        Icon(coordinateSystem(grid={2,2}), graphics={
            Line(points={{-80,-80},{-80,68}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-80,0},{-69,34},{-62,53},{-55,68},{-50,75},{-44,79},{-38,80},{-32,76},{-27,70},{-21,59},{-15,44},{-7,21},{10,-31},{17,-50},{24,-64},{29,-73},{35,-78},{41,-81},{46,-78},{52,-71},{57,-62},{64,-47},{72,-25},{80,0},{72,-53},{59,-37},{46,-95},{34,-53},{22,-81},{10,-10},{-3,-27},{-13,63},{-26,46},{-26,48},{-38,94},{-51,49},{-59,80},{-65,18},{-75,38},{-80,0}},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{2,80},{82,20}},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid,
              textString="1",
              visible=useFirstHarmonic),
            Text(
              extent={{2,80},{82,20}},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid,
              textString="rms",
              visible=not useFirstHarmonic),
            Text(
              extent={{-150,-110},{150,-150}},
              textString="f=%f")}),      Documentation(info="<html>
<p>This block determines the total harmonic distortion (THD) over the given period <code>1/f</code>.
Consider that the input <code>u</code> consists of harmonic RMS components
<code>U<sub>1</sub></code>, <code>U<sub>2</sub></code>, <code>U<sub>3</sub></code>, etc.
The total RMS component is then determined by:</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Math/Urms.png\">
</p>

<p>
The calculation of the total harmonic distortion is based on the parameter <code>useFirstHarmonic</code>.
The default value <code>useFirstHarmonic = true</code> represents the <strong>standard</strong> THD calculation used in
<a href=\"http://www.electropedia.org/iev/iev.nsf/display?openform&amp;ievref=551-20-13\">electrical engineering</a>.
The non-default value <code>useFirstHarmonic = false</code>
calculates the THD typically used for the assessment of audio signals.
</p>

<p>
If <code>useFirstHarmonic = true</code>, the total higher harmonic content (harmonic order numbers &gt; 1)
refers to the RMS value of the fundamental wave:<br>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Math/THD1.png\">
</p>

<p>
If <code>useFirstHarmonic = false</code>, the total higher harmonic content (harmonic order numbers &gt; 1)
refers to the total RMS:<br>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Math/THDrms.png\">
</p>

<p>
In case of a zero input signal or within the first period of calculation, the boolean output signal
<code>valid</code> becomes <code>false</code> to indicate that the calculation result is not valid. Valid
calculations are indicated by <code>valid = true</code>.
</p>
</html>"));
    end TotalHarmonicDistortion;

    block RealFFT "Sampling and FFT of input u"
      extends Blocks.Interfaces.DiscreteBlock(final samplePeriod=1/(2*f_res*div(
            ns, 2)));
      parameter SI.Frequency f_max "Maximum frequency of interest";
      parameter SI.Frequency f_res "Frequency resolution";
      final parameter Integer ns=Modelica.Math.FastFourierTransform.realFFTsamplePoints(f_max, f_res, f_max_factor=5) "Number of samples";
      final parameter Integer nf=max(1,min(integer(ceil(f_max/f_res))+1, div(ns, 2))) "Number of frequency points";
      parameter String resultFileName="realFFT.mat" "Result file: f, abs, arg";
      output Integer info(start=0, fixed=true) "Information flag from FFT computation";
      Blocks.Interfaces.RealInput u annotation (Placement(transformation(extent
              ={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},{
                -100,20}})));
    protected
      Real buf[ns](start=zeros(ns), each fixed=true) "Input buffer";
      Real abs[nf](start=zeros(nf), each fixed=true) "FFT amplitudes";
      Real arg[nf](start=zeros(nf), each fixed=true) "FFT phases";
      Integer iTick(start=0, fixed=true) "Sample ticks";
    algorithm
      when {sampleTrigger} then
        iTick :=pre(iTick) + 1;
        if iTick <= ns then
          buf[iTick] :=u;
        end if;
      end when;
      when terminal() then
        if iTick < ns then
          assert(false, "Sampling time not sufficient! stopTime>= "+String(startTime + (ns -1)*samplePeriod));
        else
          (info, abs, arg) :=Modelica.Math.FastFourierTransform.realFFT(buf, nf);
          assert(info==0, "Error in FFT");
          Modelica.Math.FastFourierTransform.realFFTwriteToFile(startTime + (ns - 1)*samplePeriod, resultFileName, f_max, abs, arg);
        end if;
      end when;
      annotation (Documentation(info="<html>
<p>
This block samples the input signal, calculates the Fast Fourier Transform by function <a href=\"modelica://Modelica.Math.FastFourierTransform.realFFT\">Math.realFFT</a>,
and (when simulation terminates) writes the output to result file resultFileName by function <a href=\"modelica://Modelica.Math.FastFourierTransform.realFFTwriteToFile\">Math.realFFTwriteToFile</a>.
</p>
<p>
The number of sampling points as well as the samplePeriod is calculated from desired maximum frequency f_max and frequency resolution f_res.
</p>
<h4>Note</h4>
<p>
The user has to take care that enough points can be sampled before the simulation ends: startTime + (ns - 1)*samplePeriod <= stopTime.
</p>
<p>
The result file is written as mat, first column = frequency, second column = amplitudes, third column = phases. The frequency points are separated by rows with amplitude and phase = 0,
so one can plot the result directly as frequency lines.
</p>
</html>"),   Icon(graphics={    Polygon(
                points={{-80,96},{-86,80},{-74,80},{-80,96}},
                lineColor={135,135,135},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),                    Line(
              points={{-80,-92},{-80,80}},
                                       color={135,135,135}),
                                               Line(points={{-92,-80},{80,
                  -80.3976}},
              color={135,135,135}),Polygon(
                points={{96,-80.3976},{80,-74.3976},{80,-86.3976},{96,-80.3976}},
                lineColor={135,135,135},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
            Line(
              points={{-70,60},{-70,-80}},
              thickness=0.5),
            Line(
              points={{-30,-52},{-30,-80}},
              thickness=0.5),
            Line(
              points={{-10,-60},{-10,-80}},
              thickness=0.5),
            Line(
              points={{30,-68},{30,-80}},
              thickness=0.5),
            Line(
              points={{50,-70},{50,-80}},
              thickness=0.5)}));
    end RealFFT;

    block Pythagoras "Determine the hypotenuse or leg of a right triangle"
      extends Interfaces.SI2SO;
      parameter Boolean u1IsHypotenuse = false "= true, if u1 is the hypotenuse and y is one leg";
      Interfaces.BooleanOutput valid "= true, if y is a valid result" annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
    protected
      Real y2 "Square of y";
    equation
      if not u1IsHypotenuse then
        y2 = u1^2 + u2^2;
        y = sqrt(y2);
        valid = true;
      else
        y2 = u1^2 - u2^2;
        valid = y2 >= 0;
        y = if noEvent(y2 >= 0) then sqrt(y2) else 0;
      end if;

      annotation (Icon(graphics={
            Polygon(
              points={{34,-80},{34,80},{-36,-40},{34,-80}},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-100,60},{22,60}},
              pattern=LinePattern.Dash),
            Line(
              points={{34,0},{100,0}},
              pattern=LinePattern.Dash),
            Line(
              points={{-100,-60},{0,-60}},
              pattern=LinePattern.Dash),
            Line(
              visible=u1IsHypotenuse,
              points={{22,60},{34,60}},
              pattern=LinePattern.Dash),
            Line(
              visible=u1IsHypotenuse,
              points={{-12,0},{34,0}},
              pattern=LinePattern.Dash)}), Documentation(info="<html>
<p>This block determines the hypotenuse <code>y = sqrt(u1^2 + u2^2)</code>
if the boolean parameter <code>u1IsHyotenuse = false</code>.
In this case the two inputs <code>u1</code> and
<code>u2</code> are interpreted as the legs of a right triangle
and the boolean output <code>valid</code> is always equal to
<code>true</code>.</p>

<p>If <code>u1IsHyotenuse = true</code>, input <code>u1</code> is interpreted as hypotenuse and <code>u2</code>
is one of the two legs of a right triangle.
Then, the other of the two legs of the right triangle is the output, determined by
<code>y = sqrt(u1^2 - u2^2)</code>, if <code>u1^2 - u2^2 &ge; 0</code>; in this case the
boolean output <code>valid</code> is equal to <code>true</code>. In case of <code>u1^2 - u2^2 &lt; 0</code>, the
output <code>y = 0</code> and <code>valid</code> is set to <code>false</code>.</p>
</html>"));
    end Pythagoras;

    block Max "Pass through the largest signal"
      extends Interfaces.SI2SO;
    equation
      y = max(u1, u2);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,36},{90,-36}},
              textColor={160,160,164},
              textString="max()")}), Documentation(info="<html>
<p>
This block computes the output <strong>y</strong> as <em>maximum</em>
of the two Real inputs <strong>u1</strong> and <strong>u2</strong>:
</p>
<blockquote><pre>
y = <strong>max</strong> ( u1 , u2 );
</pre></blockquote>
</html>"));
    end Max;

    block Min "Pass through the smallest signal"
      extends Interfaces.SI2SO;
    equation
      y = min(u1, u2);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,36},{90,-36}},
              textColor={160,160,164},
              textString="min()")}), Documentation(info="<html>
<p>
This block computes the output <strong>y</strong> as <em>minimum</em> of
the two Real inputs <strong>u1</strong> and <strong>u2</strong>:
</p>
<blockquote><pre>
y = <strong>min</strong> ( u1 , u2 );
</pre></blockquote>
</html>"));
    end Min;

    block MinMax "Output the minimum and the maximum element of the input vector"
      extends Blocks.Icons.Block;
      parameter Integer nu(min=0) = 0 "Number of input connections"
        annotation (Dialog(connectorSizing=true), HideResult=true);
      Blocks.Interfaces.RealVectorInput u[nu]
        annotation (Placement(transformation(extent={{-120,70},{-80,-70}})));
      Blocks.Interfaces.RealOutput yMax
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
      Blocks.Interfaces.RealOutput yMin
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
    equation
      yMax = max(u);
      yMin = min(u);
      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={Text(
              extent={{-12,80},{100,40}},
              textString="yMax"), Text(
              extent={{-10,-40},{100,-80}},
              textString="yMin")}), Documentation(info="<html>
<p>
Determines the minimum and maximum element of the input vector and
provide both values as output.
</p>
</html>"));
    end MinMax;

    block LinearDependency "Output a linear combination of the two inputs"
      extends Blocks.Interfaces.SI2SO;
      parameter Real y0=0 "Initial value";
      parameter Real k1=0 "Gain of u1";
      parameter Real k2=0 "Gain of u2";
    equation
      y = y0 + k1*u1 + k2*u2;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics={
            Line(
              points={{-100,60},{100,0},{-100,-60}},
              color={0,0,127}),
            Text(
              extent={{-14,88},{94,32}},
              textString="%k1"),
            Text(
              extent={{-40,-48},{96,-96}},
              textString="%k2"),
            Text(
              extent={{-94,26},{8,-30}},
              textString="%y0")}), Documentation(info="<html>
<p>Determine the linear combination of the two inputs: <code>y = y0 + k1*u1 + k2*u2</code></p>
</html>"));
    end LinearDependency;

    block Edge "Indicates rising edge of Boolean signal"
      extends Interfaces.BooleanSISO;
    equation
      y = edge(u);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,36},{90,-36}},
              textColor={160,160,164},
              textString="edge()")}), Documentation(info="<html>
<p>
This block sets the Boolean output <strong>y</strong> to true,
when the Boolean input <strong>u</strong> shows a <em>rising edge</em>:
</p>
<blockquote><pre>
y = <strong>edge</strong>( u );
</pre></blockquote>
</html>"));
    end Edge;

    block BooleanChange "Indicates Boolean signal changing"
      extends Interfaces.BooleanSISO;
    equation
      y = change(u);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,36},{90,-36}},
              textColor={160,160,164},
              textString="change()")}), Documentation(info="<html>
<p>
This block sets the Boolean output <strong>y</strong> to true, when the
Boolean input <strong>u</strong> shows a <em>rising or falling edge</em>,
i.e., when the signal changes:
</p>
<blockquote><pre>
y = <strong>change</strong>( u );
</pre></blockquote>
</html>"));
    end BooleanChange;

    block IntegerChange "Indicates integer signal changing"
      extends Interfaces.IntegerSIBooleanSO;
    equation
      y = change(u);
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-90,36},{90,-36}},
              textColor={160,160,164},
              textString="change()")}), Documentation(info="<html>
<p>
This block sets the Boolean output <strong>y</strong> to true, when the
Integer input <strong>u</strong> changes:
</p>
<blockquote><pre>
y = <strong>change</strong>( u );
</pre></blockquote>
</html>"));
    end IntegerChange;

    annotation (Documentation(info="<html>
<p>
This package contains basic <strong>mathematical operations</strong>,
such as summation and multiplication, and basic <strong>mathematical
functions</strong>, such as <strong>sqrt</strong> and <strong>sin</strong>, as
input/output blocks. All blocks of this library can be either
connected with continuous blocks or with sampled-data blocks.
</p>
</html>",   revisions="<html>
<ul>
<li><em>August 24, 2016</em>
       by Christian Kral: added WrapAngle</li>
<li><em>October 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       New blocks added: RealToInteger, IntegerToReal, Max, Min, Edge, BooleanChange, IntegerChange.</li>
<li><em>August 7, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized (partly based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist).
</li>
</ul>
</html>"),   Icon(graphics={Line(
            points={{-80,-2},{-68.7,32.2},{-61.5,51.1},{-55.1,64.4},{-49.4,72.6},
                {-43.8,77.1},{-38.2,77.8},{-32.6,74.6},{-26.9,67.7},{-21.3,57.4},
                {-14.9,42.1},{-6.83,19.2},{10.1,-32.8},{17.3,-52.2},{23.7,-66.2},
                {29.3,-75.1},{35,-80.4},{40.6,-82},{46.2,-79.6},{51.9,-73.5},{
                57.5,-63.9},{63.9,-49.2},{72,-26.8},{80,-2}},
            color={95,95,95},
            smooth=Smooth.Bezier)}));
  end Math;

  package MathInteger
    "Library of Integer mathematical functions as input/output blocks"
    extends Modelica.Icons.Package;
  block MultiSwitch
      "Set Integer expression that is associated with the first active input signal"

    input Integer expr[nu]=fill(0, nu)
        "y = if u[i] then expr[i] elseif use_pre_as_default then pre(y) else y_default" annotation(Dialog);
    parameter Integer y_default=0
        "Default value of output y if use_pre_as_default=false, as well as pre(y) at initial time";

    parameter Boolean use_pre_as_default=true
        "= true, y holds its last value if all u[i]=false, otherwise y=y_default"
          annotation(HideResult=true, choices(checkBox=true));
    parameter Integer nu(min=0) = 0 "Number of input connections"
            annotation(Dialog(connectorSizing=true), HideResult=true);

    Blocks.Interfaces.BooleanVectorInput u[nu]
        "Set y = expr[i], if u[i] = true"
        annotation (Placement(transformation(extent={{-110,30},{-90,-30}})));
    Blocks.Interfaces.IntegerOutput y "Output depending on expression"
        annotation (Placement(transformation(extent={{300,-10},{320,10}})));

    protected
    Integer firstActiveIndex;
  initial equation
    pre(y) = y_default;
  equation
    firstActiveIndex = Modelica.Math.BooleanVectors.firstTrueIndex(
                                                    u);
    y = if firstActiveIndex > 0 then expr[firstActiveIndex] else
        if use_pre_as_default then pre(y) else y_default;
    annotation (defaultComponentName="multiSwitch1", Icon(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{300,100}}), graphics={
              Text(
                extent={{310,-25},{410,-45}},
                textString=DynamicSelect(" ", String(
                    y,
                    minimumLength=1,
                    significantDigits=0))),
              Text(
                visible=not use_pre_as_default,
                extent={{-100,-60},{300,-90}},
                textString="else: %y_default"),
              Text(
                visible=use_pre_as_default,
                extent={{-100,-50},{300,-80}},
                textString="else: pre(y)"),
              Rectangle(
                extent={{-100,-40},{300,40}},
                fillColor={255,213,170},
                fillPattern=FillPattern.Solid,
                borderPattern=BorderPattern.Raised),
              Text(
                extent={{-100,90},{300,50}},
                textString="%name",
                textColor={0,0,255}),
              Text(
                extent={{-80,15},{290,-15}},
                textString="%expr")}),
      Documentation(info="<html>
<p>
This block has a vector of Boolean input signals u[nu] and a vector of
(time varying) Integer expressions expr[nu]. The output signal y is
set to expr[i], if i is the first element in the input vector u that is true. If all input signals are
false, y is set to parameter \"y_default\" or the last value is kept, if use_pre_as_default = <strong>true</strong>.
</p>

<blockquote><pre>
// Conceptual equation (not valid Modelica)
i = 'first element of u[:] that is true';
y = <strong>if</strong> i==0 <strong>then</strong> (<strong>if</strong> use_pre_as_default <strong>then</strong> pre(y)
                                        <strong>else</strong> y_default)
    <strong>else</strong> expr[i];
</pre></blockquote>

<p>
The input connector is a vector of Boolean input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.IntegerNetwork1\">Modelica.Blocks.Examples.IntegerNetwork1</a>.
</p>

</html>"));
  end MultiSwitch;

    block Sum "Sum of Integers: y = k[1]*u[1] + k[2]*u[2] + ... + k[n]*u[n]"
       extends Blocks.Interfaces.PartialIntegerMISO;
       parameter Integer k[nu] = fill(1,nu) "Input gains";
    equation
      if size(u,1) > 0 then
         y = k*u;
      else
         y = 0;
      end if;
      annotation (Icon(graphics={Text(
                extent={{-200,-110},{200,-140}},
                textString="%k"), Text(
                extent={{-72,68},{92,-68}},
                textString="+")}), Documentation(info="<html>
<p>
This blocks computes the scalar Integer output \"y\" as sum of the elements of the
Integer input signal vector u:
</p>
<blockquote><pre>
y = k[1]*u[1] + k[2]*u[2] + ... k[N]*u[N];
</pre></blockquote>

<p>
The input connector is a vector of Integer input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.IntegerNetwork1\">Modelica.Blocks.Examples.IntegerNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to zero: y=0.
</p>
</html>"));
    end Sum;

    block Product "Product of Integer: y = u[1]*u[2]* ... *u[n]"
       extends Blocks.Interfaces.PartialIntegerMISO;
    equation
      if size(u,1) > 0 then
         y = product(u);
      else
         y = 0;
      end if;

      annotation (Icon(graphics={Text(
                extent={{-74,50},{94,-94}},
                textString="*")}), Documentation(info="<html>
<p>
This blocks computes the scalar Integer output \"y\" as product of the elements of the
Integer input signal vector u:
</p>
<blockquote><pre>
y = u[1]*u[2]* ... *u[N];
</pre></blockquote>

<p>
The input connector is a vector of Integer input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.IntegerNetwork1\">Modelica.Blocks.Examples.IntegerNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to zero: y=0.
</p>
</html>"));
    end Product;

    block TriggeredAdd
      "Add input to previous value of output, if rising edge of trigger port"
      extends Blocks.Interfaces.PartialIntegerSISO;

      parameter Boolean use_reset = false "= true, if reset port enabled"
            annotation(Evaluate=true, HideResult=true, choices(checkBox=true));
      parameter Boolean use_set = false
        "= true, if set port enabled and used as default value when reset"
            annotation(Dialog(enable=use_reset), Evaluate=true, HideResult=true, choices(checkBox=true));
      parameter Integer y_start = 0
        "Initial and reset value of y if set port is not used";

      Blocks.Interfaces.BooleanInput trigger annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={-60,-120})));
      Blocks.Interfaces.BooleanInput reset if use_reset annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=90,
            origin={60,-120})));
      Blocks.Interfaces.IntegerInput set if use_set annotation (Placement(
            transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={60,120}), iconTransformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={28,98})));
    protected
      Blocks.Interfaces.BooleanOutput local_reset annotation (HideResult=true);
      Blocks.Interfaces.IntegerOutput local_set;
    initial equation
      pre(y) = y_start;
    equation
      if use_reset then
         connect(reset, local_reset);
           if use_set then
             connect(set, local_set);
           else
             local_set = y_start;
           end if;
      else
         local_reset = false;
         local_set = 0;
      end if;

      when {trigger, local_reset} then
         y = if local_reset then local_set else pre(y) + u;
      end when;
      annotation (Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}},
            initialScale=0.06), graphics={
              Line(
                points={{-100,0},{32,76}},
                color={255,128,0},
                pattern=LinePattern.Dot),
              Line(
                points={{-100,0},{32,-20}},
                color={255,128,0},
                pattern=LinePattern.Dot),
              Line(
                points={{-54,-56},{-26,-56},{-26,-20},{32,-20},{32,76}}),
              Line(
                points={{-60,-100},{32,-20}},
                color={255,0,255},
                pattern=LinePattern.Dot),
              Text(
                visible=use_reset,
                extent={{-28,-62},{94,-86}},
                textString="reset")}),
        Documentation(info="<html>
<p>
Add input to previous value of output, if rising edge of trigger port
</p>

<p>
This block has one Integer input \"u\", one Boolean input \"trigger\",
an optional Boolean input \"reset\", an optional Integer input \"set\", and
an Integer output \"y\".
The optional inputs can be activated with the \"use_reset\" and
\"use_set\" flags, respectively.
</p>

<p>
The input \"u\" is added to the previous value of the
output \"y\" if the \"trigger\" port has a rising edge. At the start of the
simulation \"y = y_start\".
</p>

<p>
If the \"reset\" port is enabled, then the output \"y\" is reset to \"set\"
or to \"y_start\" (if the \"set\" port is not enabled), whenever the \"reset\"
port has a rising edge.
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.IntegerNetwork1\">Modelica.Blocks.Examples.IntegerNetwork1</a>.
</p>

</html>"));
    end TriggeredAdd;
    annotation (Documentation(info="<html>
<p>
This package contains basic <strong>mathematical operations</strong>
on <strong>Integer</strong> signals.
</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
              {100,100}}), graphics={Line(
            points={{-74,-66},{-46,-66},{-46,-30},{12,-30},{12,66}},
            color={255,128,0}), Line(
            points={{12,66},{70,66}},
            color={255,128,0})}));
  end MathInteger;

  package MathBoolean
    "Library of Boolean mathematical functions as input/output blocks"
    extends Modelica.Icons.Package;

  block MultiSwitch
      "Set Boolean expression that is associated with the first active input signal"

    input Boolean expr[nu]=fill(false, nu)
        "Sets y = if u[i] then expr[i] else y_default (time varying)" annotation(Dialog);
    parameter Boolean use_pre_as_default=true
        "Set true to hold last value as default (y_default = pre(y))"
          annotation(HideResult=true, choices(checkBox=true));
    parameter Boolean y_default=false
        "Default value of output y if all u[i] = false"
                                                      annotation(Dialog(enable = not use_pre_as_default));

    parameter Integer nu(min=0) = 0 "Number of input connections"
            annotation(Dialog(connectorSizing=true), HideResult=true);

    Blocks.Interfaces.BooleanVectorInput u[nu]
        "Set y = expr[i], if u[i] = true"
        annotation (Placement(transformation(extent={{-110,30},{-90,-30}})));
    Blocks.Interfaces.BooleanOutput y "Output depending on expression"
        annotation (Placement(transformation(extent={{300,-10},{320,10}})));

    protected
    Integer firstActiveIndex;
  initial equation
    pre(y) = y_default;
  equation
      firstActiveIndex =
        Modelica.Math.BooleanVectors.firstTrueIndex(
                                     u);
     y = if firstActiveIndex == 0 then (if use_pre_as_default then pre(y) else y_default) else
                                       expr[firstActiveIndex];
    annotation (
      defaultComponentName="set1",
      Icon(coordinateSystem(
          preserveAspectRatio=true,
          extent={{-100,-100},{300,100}}), graphics={
              Text(
                visible=not use_pre_as_default,
                extent={{-100,-60},{300,-90}},
                textString="else: %y_default"),
              Text(
                visible=use_pre_as_default,
                extent={{-100,-60},{300,-90}},
                textString="else: pre(y)"),
              Text(
                extent={{-99,99},{300,59}},
                textString="%name",
                textColor={0,0,255}),
              Rectangle(
                extent={{-100,-51},{300,50}},
                lineColor={255,127,0},
                lineThickness=5.0,
                fillColor={210,210,210},
                fillPattern=FillPattern.Solid,
                borderPattern=BorderPattern.Raised),
              Text(
                extent={{-84,16},{273,-15}},
                textString="%expr"),
              Ellipse(
                extent={{275,8},{289,-6}},
                lineColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
                fillColor=DynamicSelect({235,235,235}, if y then {0,255,0} else {235,235,235}),
                fillPattern=FillPattern.Solid)}),
      Documentation(info="<html>
<p>
The block has a vector of Boolean input signals u[nu] and a vector of
(time varying) Boolean expressions expr[:]. The output signal y is
set to expr[i], if i is the first element in the input vector u that is true.
If all input signals are false, y is set to parameter \"y_default\" or the
previous value of y is kept if parameter use_pre_as_default = <strong>true</strong>:
</p>

<blockquote><pre>
// Conceptual equation (not valid Modelica)
i = 'first element of u[:] that is true';
y = <strong>if</strong> i==0 <strong>then</strong> (<strong>if</strong> use_pre_as_default <strong>then</strong> pre(y)
                                        <strong>else</strong> y_default)
    <strong>else</strong> expr[i];
</pre></blockquote>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

</html>"));
  end MultiSwitch;

    block And "Logical 'and': y = u[1] and u[2] and ... and u[nu]"
      extends Blocks.Interfaces.PartialBooleanMISO;

    equation
      y = Modelica.Math.BooleanVectors.andTrue(
                                u);
      annotation (defaultComponentName="and1", Icon(graphics={Text(
                extent={{-76,40},{60,-40}},
                textString="and")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if all inputs are <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>

<p>
The input connector is a vector of Boolean input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to <strong>true</strong>: y=true.
</p>
</html>"));
    end And;

    block Or "Logical 'or': y = u[1] or u[2] or ... or u[nu]"
      extends Blocks.Interfaces.PartialBooleanMISO;

    equation
      y = Modelica.Math.BooleanVectors.anyTrue(
                                u);
      annotation (defaultComponentName="or1", Icon(graphics={Text(
                extent={{-80,40},{60,-40}},
                textString="or")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if at least one input is <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>

<p>
The input connector is a vector of Boolean input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to <strong>false</strong>: y=false.
</p>

</html>"));
    end Or;

    block Xor
      "Logical 'xor': y = oneTrue(u)  (y is true, if exactly one element of u is true, otherwise it is false)"
      extends Blocks.Interfaces.PartialBooleanMISO;

    equation
      y = Modelica.Math.BooleanVectors.oneTrue(
                                u);
      annotation (defaultComponentName="xor1", Icon(graphics={Text(
                extent={{-80,40},{60,-40}},
                textString="xor")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if exactly one input is <strong>true</strong>, otherwise
the output is <strong>false</strong>.
</p>

<p>
The input connector is a vector of Boolean input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to <strong>false</strong>: y=false.
</p>

</html>"));
    end Xor;

    block Nand "Logical 'nand': y = not ( u[1] and u[2] and ... and u[nu] )"
      extends Blocks.Interfaces.PartialBooleanMISO;

    equation
      y = not Modelica.Math.BooleanVectors.andTrue(
                                    u);
      annotation (defaultComponentName="nand1", Icon(graphics={Text(
                extent={{-78,36},{64,-30}},
                textString="nand")}),
        Documentation(info="<html>
<p>
The output is <strong>true</strong> if at least one input is <strong>false</strong>, otherwise
the output is <strong>false</strong>.
</p>

<p>
The input connector is a vector of Boolean input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to <strong>false</strong>: y=false.
</p>

</html>"));
    end Nand;

    block Nor "Logical 'nor': y = not ( u[1] or u[2] or ... or u[nu] )"
      extends Blocks.Interfaces.PartialBooleanMISO;

    equation
      y = not Modelica.Math.BooleanVectors.anyTrue(
                                    u);
      annotation (defaultComponentName="nor1", Icon(graphics={Text(
                extent={{-80,40},{60,-40}},
                textString="nor")}),
        Documentation(info="<html>
<p>
The output is <strong>false</strong> if at least one input is <strong>true</strong>, otherwise
the output is <strong>true</strong>.
</p>

<p>
The input connector is a vector of Boolean input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

<p>
If no connection to the input connector \"u\" is present,
the output is set to <strong>false</strong>: y=false.
</p>
</html>"));
    end Nor;

    block Not "Logical 'not': y = not u"
      extends Blocks.Interfaces.PartialBooleanSISO_small;

    equation
      y = not u;
      annotation (defaultComponentName="not1", Icon(graphics={Text(
                extent={{-98,40},{42,-40}},
                textString="not")}),
        Documentation(info="<html>
<p>
The output is <strong>false</strong> if at least one input is <strong>true</strong>, otherwise
the output is <strong>true</strong>.
</p>

<p>
The input connector is a vector of Boolean input signals.
When a connection line is drawn, the dimension of the input
vector is enlarged by one and the connection is automatically
connected to this new free index (thanks to the
connectorSizing annotation).
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>
</html>"));
    end Not;

    block RisingEdge
      "Output y is true, if the input u has a rising edge, otherwise it is false (y = edge(u))"
       parameter Boolean pre_u_start = false "Value of pre(u) at initial time";
       extends Blocks.Interfaces.PartialBooleanSISO_small;
    initial equation
      pre(u) = pre_u_start;
    equation
      y = edge(u);
          annotation (defaultComponentName="rising1", Icon(graphics={Line(points=
                    {{-80,-68},{-36,-68},{-36,-24},{22,-24},{22,-68},{66,-68}}), Line(points={{-80,32},{-36,32},{-36,76},{-36,76},
                    {-36,32},{66,32}}, color={255,0,255})}),
                                     Documentation(info="<html>
<p>
A rising edge of the Boolean input u results in y = <strong>true</strong> at this
time instant. At all other time instants, y = <strong>false</strong>.
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

</html>"));
    end RisingEdge;

    block FallingEdge
      "Output y is true, if the input u has a falling edge, otherwise it is false (y = edge(not u))"
       parameter Boolean pre_u_start = false "Value of pre(u) at initial time";
       extends Blocks.Interfaces.PartialBooleanSISO_small;
    protected
      Boolean not_u = not u annotation(HideResult=true);
    initial equation
      pre(not_u) = not pre_u_start;
    equation
      y = edge(not_u);
          annotation (defaultComponentName="falling1", Icon(graphics={Line(points=
                   {{-80,-68},{-36,-68},{-36,-24},{22,-24},{22,-68},{66,-68}}), Line(points={{-80,32},{24,32},{24,76},{24,76},{
                    24,32},{66,32}}, color={255,0,255})}),
                                     Documentation(info="<html>
<p>
A falling edge of the Boolean input u results in y = <strong>true</strong> at this
time instant. At all other time instants, y = <strong>false</strong>.
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

</html>"));
    end FallingEdge;

    block ChangingEdge
      "Output y is true, if the input u has either a rising or a falling edge and otherwise it is false (y=change(u))"
       parameter Boolean pre_u_start = false "Value of pre(u) at initial time";
       extends Blocks.Interfaces.PartialBooleanSISO_small;
    initial equation
      pre(u) = pre_u_start;
    equation
      y = change(u);
          annotation (defaultComponentName="changing1", Icon(graphics={
              Line(points={{-80,-68},{-36,-68},{-36,-24},{22,-24},{22,-68},{66,-68}}),
              Line(points={{-80,32},{-36,32},{-36,76},{-36,76},{-36,32},{66,32}},
                  color={255,0,255}),
              Line(
                points={{24,32},{24,76}},
                color={255,0,255})}),Documentation(info="<html>
<p>
A changing edge, i.e., either rising or falling,
of the Boolean input u results in y = <strong>true</strong> at this
time instant. At all other time instants, y = <strong>false</strong>.
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

</html>"));
    end ChangingEdge;

    block OnDelay
      "Delay a rising edge of the input, but do not delay a falling edge."
          extends Blocks.Interfaces.PartialBooleanSISO_small;
          parameter SI.Time delayTime "Delay time";

    protected
          Boolean delaySignal(start=false,fixed=true);
          discrete SI.Time t_next;
    initial equation
          pre(u) = false;
          pre(t_next) = time - 1;
    algorithm
          when initial() then
             delaySignal := u;
             t_next := time - 1;
          elsewhen u then
             delaySignal := true;
             t_next := time + delayTime;
          elsewhen not u then
             delaySignal := false;
             t_next := time - 1;
          end when;
    equation
          if delaySignal then
             y = time >= t_next;
          else
             y = false;
          end if;
          annotation (Icon(graphics={
              Text(
                extent={{-250,-120},{250,-150}},
                textString="%delayTime s"),
              Line(points={{-80,-66},{-60,-66},{-60,-22},{38,-22},{38,-66},{66,-66}}),
              Line(points={{-80,32},{-4,32},{-4,76},{38,76},{38,32},{66,32}},
                  color={255,0,255})}),
                                     Documentation(info="<html>
<p>
A rising edge of the Boolean input u gives a delayed output.
A falling edge of the input is immediately given to the output.
</p>

<p>
Simulation results of a typical example with a delay time of 0.1 s
is shown in the next figure.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/MathBoolean/OnDelay1.png\"
     alt=\"OnDelay1.png\">
<br>
<img src=\"modelica://Modelica/Resources/Images/Blocks/MathBoolean/OnDelay2.png\"
     alt=\"OnDelay2.png\">
</p>

<p>
The usage is demonstrated, e.g., in example
<a href=\"modelica://Modelica.Blocks.Examples.BooleanNetwork1\">Modelica.Blocks.Examples.BooleanNetwork1</a>.
</p>

</html>"));
    end OnDelay;

    annotation (Documentation(info="<html>
<p>
This package contains basic <strong>mathematical operations</strong>
on <strong>Boolean</strong> signals.
</p>

<p>
The new features are:
</p>

<ul>
<li> If useful, blocks may have an arbitrary number of inputs (e.g., \"And\" block with 2,3,4,...
     Boolean inputs). This is based on the \"connectorSizing\" annotation which
     allows a tool to conveniently handle vectors of connectors.</li>

<li> The blocks are smaller in size, so that the diagram area is better
     utilized for trivial blocks such as \"And\" or \"Or\".</li>

</ul>

</html>"),   Icon(graphics={Line(points={{-80,-16},{-4,-16},{-4,28},{38,28},{38,
                -16},{66,-16}}, color={255,0,255})}));
  end MathBoolean;

  package Nonlinear
    "Library of discontinuous or non-differentiable algebraic control blocks"
    import Blocks.Interfaces;
    extends Modelica.Icons.Package;

        block Limiter "Limit the range of a signal"
          parameter Real uMax(start=1) "Upper limits of input signals";
          parameter Real uMin= -uMax "Lower limits of input signals";
          parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
            annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
          parameter Types.LimiterHomotopy homotopyType =Blocks.Types.LimiterHomotopy.Linear
                                                                                                      "Simplified model for homotopy-based initialization"
            annotation (Evaluate=true, Dialog(group="Initialization"));
          extends Interfaces.SISO;
    protected
          Real simplifiedExpr "Simplified expression for homotopy-based initialization";

        equation
          assert(uMax >= uMin, "Limiter: Limits must be consistent. However, uMax (=" + String(uMax) +
                               ") < uMin (=" + String(uMin) + ")");
          simplifiedExpr = (if homotopyType == Types.LimiterHomotopy.Linear then u
                            else if homotopyType == Types.LimiterHomotopy.UpperLimit then uMax
                            else if homotopyType == Types.LimiterHomotopy.LowerLimit then uMin
                            else 0);
          if strict then
            if homotopyType == Types.LimiterHomotopy.NoHomotopy then
              y = smooth(0, noEvent(if u > uMax then uMax else if u < uMin then uMin else u));
            else
              y = homotopy(actual = smooth(0, noEvent(if u > uMax then uMax else if u < uMin then uMin else u)),
                           simplified=simplifiedExpr);
            end if;
          else
            if homotopyType == Types.LimiterHomotopy.NoHomotopy then
              y = smooth(0,if u > uMax then uMax else if u < uMin then uMin else u);
            else
              y = homotopy(actual = smooth(0,if u > uMax then uMax else if u < uMin then uMin else u),
                           simplified=simplifiedExpr);
            end if;
          end if;
          annotation (
            Documentation(info="<html>
<p>
The Limiter block passes its input signal as output signal
as long as the input is within the specified upper and lower
limits. If this is not the case, the corresponding limits are passed
as output.
</p>
<p>
The parameter <code>homotopyType</code> in the Advanced tab specifies the
simplified behaviour if homotopy-based initialization is used:
</p>
<ul>
<li><code>NoHomotopy</code>: the actual expression with limits is used</li>
<li><code>Linear</code>: a linear behaviour y = u is assumed (default option)</li>
<li><code>UpperLimit</code>: it is assumed that the output is stuck at the upper limit u = uMax</li>
<li><code>LowerLimit</code>: it is assumed that the output is stuck at the lower limit u = uMin</li>
</ul>
<p>
If it is known a priori in which region the input signal will be located, this option can help
a lot by removing one strong nonlinearity from the initialization problem.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{0,-90},{0,68}}, color={192,192,192}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,-8},{68,8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-70},{-50,-70},{50,70},{80,70}}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="uMax=%uMax"),
            Line(
              visible=strict,
              points={{50,70},{80,70}},
              color={255,0,0}),
            Line(
              visible=strict,
              points={{-80,-70},{-50,-70}},
              color={255,0,0})}));
        end Limiter;

    block VariableLimiter "Limit the range of a signal with variable limits"
      extends Interfaces.SISO;
      parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
        annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
      parameter Types.VariableLimiterHomotopy homotopyType =Blocks.Types.VariableLimiterHomotopy.Linear
                                                                                                                  "Simplified model for homotopy-based initialization"
        annotation (Evaluate=true, Dialog(group="Initialization"));
      parameter Real ySimplified = 0 "Fixed value of output in simplified model"
        annotation (Dialog(tab="Advanced", enable=homotopyType == Blocks.Types.VariableLimiterHomotopy.Fixed));
      Interfaces.RealInput limit1
        "Connector of Real input signal used as maximum of input u"
        annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
      Interfaces.RealInput limit2
        "Connector of Real input signal used as minimum of input u"
        annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
    protected
      Real simplifiedExpr "Simplified expression for homotopy-based initialization";
    equation
      assert(limit1 >= limit2, "Input signals are not consistent: limit1 < limit2");
      simplifiedExpr = (if homotopyType == Types.VariableLimiterHomotopy.Linear then u
                        else if homotopyType == Types.VariableLimiterHomotopy.Fixed then ySimplified
                        else 0);
      if strict then
        if homotopyType == Types.VariableLimiterHomotopy.NoHomotopy then
          y = smooth(0, noEvent(if u > limit1 then limit1 else if u < limit2 then limit2 else u));
        else
          y = homotopy(actual = smooth(0, noEvent(if u > limit1 then limit1 else if u < limit2 then limit2 else u)),
                       simplified=simplifiedExpr);
        end if;
      else
        if homotopyType == Types.VariableLimiterHomotopy.NoHomotopy then
          y = smooth(0,if u > limit1 then limit1 else if u < limit2 then limit2 else u);
        else
          y = homotopy(actual = smooth(0,if u > limit1 then limit1 else if u < limit2 then limit2 else u),
                       simplified=simplifiedExpr);
        end if;
      end if;

      annotation (
        Documentation(info="<html>
<p>
The Limiter block passes its input signal as output signal
as long as the input is within the upper and lower
limits specified by the two additional inputs limit1 and
limit2. If this is not the case, the corresponding limit
is passed as output.
</p>
<p>
The parameter <code>homotopyType</code> in the Advanced tab specifies the
simplified behaviour if homotopy-based initialization is used:
</p>
<ul>
<li><code>NoHomotopy</code>: the actual expression with limits is used</li>
<li><code>Linear</code>: a linear behaviour y = u is assumed (default option)</li>
<li><code>Fixed</code>: it is assumed that the output is fixed at the value <code>ySimplified</code></li>
</ul>
<p>
If it is known a priori in which region the input signal will be located, this option can help
a lot by removing one strong nonlinearity from the initialization problem.
</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{0,-90},{0,68}}, color={192,192,192}),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,-8},{68,8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-70},{-50,-70},{50,70},{80,70}}),
            Line(points={{-100,80},{66,80},{66,70}}, color={0,0,127}),
            Line(points={{-100,-80},{-64,-80},{-64,-70}}, color={0,0,127}),
            Polygon(points={{-64,-70},{-66,-74},{-62,-74},{-64,-70}}, lineColor={
                  0,0,127}),
            Polygon(points={{66,70},{64,74},{68,74},{66,70}}, lineColor={0,0,127}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              visible=strict,
              points={{50,70},{80,70}},
              color={255,0,0}),
            Line(
              visible=strict,
              points={{-80,-70},{-50,-70}},
              color={255,0,0})}));
    end VariableLimiter;

    block SlewRateLimiter "Limits the slew rate of a signal"
      extends Blocks.Interfaces.SISO;
      import Modelica.Constants.small;
      parameter Real Rising( min= small) = 1
        "Maximum rising slew rate [+small..+inf) [1/s]";
      parameter Real Falling(max=-small) = -Rising
        "Maximum falling slew rate (-inf..-small] [1/s]";
      parameter SI.Time Td(min=small) = 0.001
        "Derivative time constant";
      parameter Blocks.Types.Init initType=Blocks.Types.Init.SteadyState
        "Type of initialization (SteadyState implies y = u)"
        annotation (Evaluate=true, Dialog(group="Initialization"));
      parameter Real y_start=0 "Initial or guess value of output (= state)"
        annotation (Dialog(group="Initialization"));
      parameter Boolean strict=false "= true, if strict limits with noEvent(..)"
        annotation (Evaluate=true, choices(checkBox=true), Dialog(tab="Advanced"));
    protected
      Real val=(u-y)/Td;
    initial equation
      if initType == Blocks.Types.Init.SteadyState then
        y = u;
      elseif initType == Blocks.Types.Init.InitialState or initType == Blocks.Types.Init.InitialOutput
           then
        y = y_start;
      end if;
    equation
      if strict then
        der(y) = smooth(1, (if noEvent(val<Falling) then Falling else if noEvent(val>Rising) then Rising else val));
      else
        der(y) = if val<Falling then Falling else if val>Rising then Rising else val;
      end if;
      annotation (Icon(graphics={
        Line(points={{-90,0},{68,0}}, color={192,192,192}),
        Line(points={{0,-90},{0,68}}, color={192,192,192}),
        Polygon(
          points={{0,90},{-8,68},{8,68},{0,90}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{90,0},{68,-8},{68,8},{90,0}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-50,-70},{50,70}}),
        Line(
          visible=strict,
          points={{50,70},{-50,-70}},
          color={255,0,0})}),
    Documentation(info="<html>
<p>The <code>SlewRateLimiter</code> block limits the slew rate of its input signal in the range of <code>[Falling, Rising]</code>.</p>
<p>To ensure this for arbitrary inputs and in order to produce a differential output, the input is numerically differentiated
with derivative time constant <code>Td</code>. Smaller time constant <code>Td</code> means nearer ideal derivative.</p>
<p><em>Note: The user has to choose the derivative time constant according to the nature of the input signal.</em></p>
</html>",
    revisions="<html>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<th>Revision</th>
<th>Date</th>
<th>Author</th>
<th>Comment</th>
</tr>
<tr>
<td>4954</td>
<td>2012-03-02</td>
<td>A. Haumer &amp; D. Winkler</td>
<td><p>Initial version based on discussion in ticket <a href=\"https://github.com/modelica/ModelicaStandardLibrary/issues/529\">#529</a></p></td>
</tr>
</table>
</html>"));
    end SlewRateLimiter;

        block DeadZone "Provide a region of zero output"
          parameter Real uMax(start=1) "Upper limits of dead zones";
          parameter Real uMin=-uMax "Lower limits of dead zones";
          extends Interfaces.SISO;

        equation
          assert(uMax >= uMin, "DeadZone: Limits must be consistent. However, uMax (=" + String(uMax) +
                               ") < uMin (=" + String(uMin) + ")");

          y = homotopy(actual=smooth(0,if u > uMax then u - uMax else if u < uMin then u - uMin else 0), simplified=u);

          annotation (
            Documentation(info="<html>
<p>
The DeadZone block defines a region of zero output.
</p>
<p>
If the input is within uMin ... uMax, the output
is zero. Outside of this zone, the output is a linear
function of the input with a slope of 1.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{0,-90},{0,68}}, color={192,192,192}),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,-8},{68,8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-60},{-20,0},{20,0},{80,60}}),
            Text(
              extent={{-150,-150},{150,-110}},
              textColor={160,160,164},
              textString="uMax=%uMax")}));
        end DeadZone;

    block FixedDelay "Delay block with fixed DelayTime"
      extends Blocks.Interfaces.SISO;
      parameter SI.Time delayTime(start=1)
        "Delay time of output with respect to input signal";

    equation
      y = delay(u, delayTime);
      annotation (
        Documentation(info="<html>
<p>
The Input signal is delayed by a given time instant, or more precisely:
</p>
<blockquote><pre>
y = u(time - delayTime) for time &gt; time.start + delayTime
  = u(time.start)       for time &le; time.start + delayTime
</pre></blockquote>
</html>"),   Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Text(
          extent={{8.0,-142.0},{8.0,-102.0}},
          textString="delayTime=%delayTime"),
        Line(
          points={{-92.0,0.0},{-80.7,34.2},{-73.5,53.1},{-67.1,66.4},{-61.4,74.6},{-55.8,79.1},{-50.2,79.8},{-44.6,76.6},{-38.9,69.7},{-33.3,59.4},{-26.9,44.1},{-18.83,21.2},{-1.9,-30.8},{5.3,-50.2},{11.7,-64.2},{17.3,-73.1},{23.0,-78.4},{28.6,-80.0},{34.2,-77.6},{39.9,-71.5},{45.5,-61.9},{51.9,-47.2},{60.0,-24.8},{68.0,0.0}},
          color={0,0,127},
          smooth=Smooth.Bezier),
        Line(
          points={{-62.0,0.0},{-50.7,34.2},{-43.5,53.1},{-37.1,66.4},{-31.4,74.6},{-25.8,79.1},{-20.2,79.8},{-14.6,76.6},{-8.9,69.7},{-3.3,59.4},{3.1,44.1},{11.17,21.2},{28.1,-30.8},{35.3,-50.2},{41.7,-64.2},{47.3,-73.1},{53.0,-78.4},{58.6,-80.0},{64.2,-77.6},{69.9,-71.5},{75.5,-61.9},{81.9,-47.2},{90.0,-24.8},{98.0,0.0}},
          color={160,160,164},
          smooth=Smooth.Bezier)}));
    end FixedDelay;

    block PadeDelay
      "Pade approximation of delay block with fixed delayTime (use balance=true; this is not the default to be backwards compatible)"
      extends Blocks.Interfaces.SISO;
      parameter SI.Time delayTime(start=1)
        "Delay time of output with respect to input signal";
      parameter Integer n(min=1) = 1 "Order of Pade delay";
      parameter Integer m(min=1,max=n) = n
        "Order of numerator (usually m=n, or m=n-1)";
      parameter Boolean balance=false
        "= true, if state space system is balanced (highly recommended), otherwise textbook version"
        annotation(choices(checkBox=true));
      final output Real x[n]
        "State of transfer function from controller canonical form (balance=false), or balanced controller canonical form (balance=true)";

    protected
      parameter Real a1[n]( each fixed=false) "First row of A";
      parameter Real b11(        fixed=false) "= B[1,1]";
      parameter Real c[n](  each fixed=false) "C row matrix";
      parameter Real d(          fixed=false) "D matrix";
      parameter Real s[n-1](each fixed=false) "State scaling";

    function padeCoefficients2
      extends Modelica.Icons.Function;
      input Real T "Delay time";
      input Integer n "Order of denominator";
      input Integer m "Order of numerator";
      input Boolean balance=false;
      output Real a1[n] "First row of A";
      output Real b11 "= B[1,1]";
      output Real c[n] "C row matrix";
      output Real d "D matrix";
      output Real s[n-1] "Scaling such that x[i] = s[i-1]*x[i-1], i > 1";
      protected
      Real b[m + 1] "Numerator coefficients of transfer function";
      Real a[n + 1] "Denominator coefficients of transfer function";
      Real nm;
      Real bb[n + 1];
      Real A[n,n];
      Real B[n,1];
      Real C[1,n];
      Real A2[n,n] = zeros(n,n);
      Real B2[n,1] = zeros(n,1);
      Real C2[1,n] "C matrix";
      Integer nb = m+1;
      Integer na = n+1;
      Real sx[n];
    algorithm
      a[1] := 1;
      b[1] := 1;
      nm := n + m;

      for i in 1:n loop
        a[i + 1] := a[i]*(T*((n - i + 1)/(nm - i + 1))/i);
        if i <= m then
          b[i + 1] := -b[i]*(T*((m - i + 1)/(nm - i + 1))/i);
        end if;
      end for;

      b  := b[m + 1:-1:1];
      a  := a[n + 1:-1:1];
      bb := vector([zeros(n-m, 1); b]);
      d  := bb[1]/a[1];

      if balance then
        A2[1,:] := -a[2:na]/a[1];
        B2[1,1] := 1/a[1];
        for i in 1:n-1 loop
           A2[i+1,i] :=1;
        end for;
        C2[1,:] := bb[2:na] - d*a[2:na];
        (sx,A,B,C) :=Modelica.Math.Matrices.balanceABC(A2,B2,C2);
        for i in 1:n-1 loop
           s[i] := sx[i]/sx[i+1];
        end for;
        a1  := A[1,:];
        b11 := B[1,1];
        c   := vector(C);
      else
         s  := ones(n-1);
        a1  := -a[2:na]/a[1];
        b11 :=  1/a[1];
        c   := bb[2:na] - d*a[2:na];
      end if;
    end padeCoefficients2;

    equation
      der(x[1]) = a1*x + b11*u;
      if n > 1 then
         der(x[2:n]) = s.*x[1:n-1];
      end if;
      y = c*x + d*u;

    initial equation
      (a1,b11,c,d,s) = padeCoefficients2(delayTime, n, m, balance);

      if balance then
         der(x) = zeros(n);
      else
         // In order to be backwards compatible
         x[n] = u;
      end if;
      annotation (
        Documentation(info="<html>
<p>
The Input signal is delayed by a given time instant, or more precisely:
</p>
<blockquote><pre>
y = u(time - delayTime) for time &gt; time.start + delayTime
  = u(time.start)       for time &le; time.start + delayTime
</pre></blockquote>
<p>
The delay is approximated by a Pade approximation, i.e., by
a transfer function
</p>
<blockquote><pre>
        b[1]*s^m + b[2]*s^[m-1] + ... + b[m+1]
y(s) = --------------------------------------------- * u(s)
        a[1]*s^n + a[2]*s^[n-1] + ... + a[n+1]
</pre></blockquote>
<p>
where the coefficients b[:] and a[:] are calculated such that the
coefficients of the Taylor expansion of the delay exp(-T*s) around s=0
are identical up to order n+m.
</p>
<p>
The main advantage of this approach is that the delay is
approximated by a linear differential equation system, which
is continuous and continuously differentiable. For example, it
is uncritical to linearize a system containing a Pade-approximated
delay.
</p>
<p>
The standard text book version uses order \"m=n\", which is
also the default setting of this block. The setting
\"m=n-1\" may yield a better approximation in certain cases.
</p>

<p>
It is strongly recommended to always set parameter <strong>balance</strong> = true,
in order to arrive at a much better reliable numerical computation.
This is not the default, in order to be backwards compatible, so you have
to explicitly set it. Besides better numerics, also all states are initialized
with <strong>balance</strong> = true (in steady-state, so der(x)=0). Longer explanation:
</p>

<p>
By default the transfer function of the Pade approximation is implemented
in controller canonical form. This results in coefficients of the A-matrix in
the order of 1 up to the order of O(1/delayTime)^n. For already modest values
of delayTime and n, this gives largely varying coefficients (for example delayTime=0.001 and n=4
results in coefficients between 1 and 1e12). In turn, this results
in a large norm of the system matrix [A,B;C,D] and therefore in unreliable
numerical computations. When setting parameter <strong>balance</strong> = true, a state
transformation is performed that considerably reduces the norm of the system matrix.
This is performed without introducing round-off errors. For details see
function <a href=\"modelica://Modelica.Math.Matrices.balanceABC\">balanceABC</a>.
As a result, both the simulation of the PadeDelay block, and especially
its linearization becomes more reliable.
</p>

<h5>Literature</h5>
<p>Otto Foellinger: Regelungstechnik, 8. Auflage,
chapter 11.9, page 412-414, Huethig Verlag Heidelberg, 1994
</p>
</html>",     revisions="<html>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\"><tr>
<th>Date</th>
<th>Author</th>
<th>Comment</th>
</tr>
<tr>
<td>2015-01-05</td>
<td>Martin Otter (DLR-SR)</td>
<td>Introduced parameter balance=true and a new implementation
 of the PadeDelay block with an optional, more reliable numerics</td>
</tr>
</table>
</html>"),   Icon(
        coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}}),
          graphics={
        Text(extent={{8.0,-142.0},{8.0,-102.0}},
          textString="delayTime=%delayTime"),
        Line(points={{-94.0,0.0},{-82.7,34.2},{-75.5,53.1},{-69.1,66.4},{-63.4,74.6},{-57.8,79.1},{-52.2,79.8},{-46.6,76.6},{-40.9,69.7},{-35.3,59.4},{-28.9,44.1},{-20.83,21.2},{-3.9,-30.8},{3.3,-50.2},{9.7,-64.2},{15.3,-73.1},{21.0,-78.4},{26.6,-80.0},{32.2,-77.6},{37.9,-71.5},{43.5,-61.9},{49.9,-47.2},{58.0,-24.8},{66.0,0.0}},
          color={0,0,127},
          smooth=Smooth.Bezier),
        Line(points={{-72.0,0.0},{-60.7,34.2},{-53.5,53.1},{-47.1,66.4},{-41.4,74.6},{-35.8,79.1},{-30.2,79.8},{-24.6,76.6},{-18.9,69.7},{-13.3,59.4},{-6.9,44.1},{1.17,21.2},{18.1,-30.8},{25.3,-50.2},{31.7,-64.2},{37.3,-73.1},{43.0,-78.4},{48.6,-80.0},{54.2,-77.6},{59.9,-71.5},{65.5,-61.9},{71.9,-47.2},{80.0,-24.8},{88.0,0.0}},
          color={160,160,164},
          smooth=Smooth.Bezier),
        Text(textColor={160,160,164},
          extent={{-10.0,38.0},{100.0,100.0}},
          textString="m=%m"),
        Text(textColor={160,160,164},
          extent={{-98.0,-96.0},{6.0,-34.0}},
          textString="n=%n"),
        Text(visible=balance, textColor={160,160,164},
          extent={{-96,-20},{98,22}},
              textString="balanced")}));
    end PadeDelay;

    block VariableDelay "Delay block with variable DelayTime"
      extends Blocks.Interfaces.SISO;
      parameter SI.Duration delayMax(min=0, start=1) "Maximum delay time";

      Blocks.Interfaces.RealInput delayTime
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
    equation
      y = delay(u, delayTime, delayMax);
      annotation (
        Documentation(info="<html>
<p>
The Input signal is delayed by a given time instant, or more precisely:
</p>
<blockquote><pre>
y = u(time - delayTime) for time &gt; time.start + delayTime
  = u(time.start)       for time &le; time.start + delayTime
</pre></blockquote>
<p>
where delayTime is an additional input signal which must follow
the following relationship:
</p>
<blockquote><pre>
0 &le; delayTime &le; delayMax
</pre></blockquote>
</html>"),
      Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
        Text(extent={{-100.0,-148.0},{100.0,-108.0}},
          textString="delayMax=%delayMax"),
        Line(points={{-92.0,0.0},{-80.7,34.2},{-73.5,53.1},{-67.1,66.4},{-61.4,74.6},{-55.8,79.1},{-50.2,79.8},{-44.6,76.6},{-38.9,69.7},{-33.3,59.4},{-26.9,44.1},{-18.83,21.2},{-1.9,-30.8},{5.3,-50.2},{11.7,-64.2},{17.3,-73.1},{23.0,-78.4},{28.6,-80.0},{34.2,-77.6},{39.9,-71.5},{45.5,-61.9},{51.9,-47.2},{60.0,-24.8},{68.0,0.0}},
          color={0,0,127},
          smooth=Smooth.Bezier),
        Line(points={{-64.0,0.0},{-52.7,34.2},{-45.5,53.1},{-39.1,66.4},{-33.4,74.6},{-27.8,79.1},{-22.2,79.8},{-16.6,76.6},{-10.9,69.7},{-5.3,59.4},{1.1,44.1},{9.17,21.2},{26.1,-30.8},{33.3,-50.2},{39.7,-64.2},{45.3,-73.1},{51.0,-78.4},{56.6,-80.0},{62.2,-77.6},{67.9,-71.5},{73.5,-61.9},{79.9,-47.2},{88.0,-24.8},{96.0,0.0}},
          smooth=Smooth.Bezier),
        Polygon(fillPattern=FillPattern.Solid,
          lineColor={0,0,127},
          fillColor={0,0,127},
          points={{6.0,4.0},{-14.0,-2.0},{-6.0,-12.0},{6.0,4.0}}),
        Line(color={0,0,127},
          points={{-100.0,-60.0},{-76.0,-60.0},{-8.0,-6.0}})}));
    end VariableDelay;

        annotation (
          Documentation(info="<html>
<p>
This package contains <strong>discontinuous</strong> and
<strong>non-differentiable, algebraic</strong> input/output blocks.
</p>
</html>",   revisions="<html>
<ul>
<li><em>October 21, 2002</em>
       by Christian Schweiger:<br>
       New block VariableLimiter added.</li>
<li><em>August 22, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.
</li>
</ul>
</html>"),   Icon(graphics={Line(points={{-80,-66},{-26,-66},{28,52},{88,52}},
              color={95,95,95})}));
  end Nonlinear;

  package Routing "Library of blocks to combine and extract signals"
    extends Modelica.Icons.Package;

    block Replicator "Signal replicator"
      extends Blocks.Interfaces.SIMO;
    equation
      y = fill(u, nout);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-100,0},{-6,0}}, color={0,0,127}),
            Line(points={{100,0},{10,0}}, color={0,0,127}),
            Line(points={{0,0},{100,10}}, color={0,0,127}),
            Line(points={{0,0},{100,-10}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This block replicates the input signal to an array of <code>nout</code> identical output signals.
</p>
</html>"));
    end Replicator;

    block IntegerReplicator "Integer signal replicator"
      extends Blocks.Icons.IntegerBlock;
      parameter Integer nout=1 "Number of outputs";
      Blocks.Interfaces.IntegerInput u "Connector of Integer input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.IntegerOutput y[nout]
        "Connector of Integer output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation

      y = fill(u, nout);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-100,0},{-6,0}}, color={255,127,0}),
            Line(points={{100,0},{10,0}}, color={255,127,0}),
            Line(points={{0,0},{100,10}}, color={255,127,0}),
            Line(points={{0,0},{100,-10}}, color={255,127,0}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={255,127,0},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This block replicates the Integer input signal to an array of <code>nout</code> identical Integer output signals.
</p>
</html>"));
    end IntegerReplicator;

    block BooleanReplicator "Boolean signal replicator"
      extends Blocks.Icons.BooleanBlock;
      parameter Integer nout=1 "Number of outputs";
      Blocks.Interfaces.BooleanInput u "Connector of Boolean input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.BooleanOutput y[nout]
        "Connector of Boolean output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation

      y = fill(u, nout);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-100,0},{-6,0}}, color={255,0,255}),
            Line(points={{100,0},{10,0}}, color={255,0,255}),
            Line(points={{0,0},{100,10}}, color={255,0,255}),
            Line(points={{0,0},{100,-10}}, color={255,0,255}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid)}),
        Documentation(info="<html>
<p>
This block replicates the Boolean input signal to an array of <code>nout</code> identical Boolean output signals.
</p>
</html>"));
    end BooleanReplicator;

  block ExtractSignal "Extract signals from an input signal vector"
    extends Blocks.Interfaces.MIMO;
    parameter Integer extract[nout]=1:nout "Extracting vector";

  equation
    for i in 1:nout loop
      y[i] = u[extract[i]];

    end for;
    annotation (
      Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-90,51},{-50,-49}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Rectangle(
              extent={{50,50},{90,-50}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Polygon(
              points={{-94.4104,1.90792},{-94.4104,-2.09208},{-90.4104,-0.0920762},
                  {-94.4104,1.90792}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-72,2},{-60.1395,12.907},{-49.1395,12.907}}, color={0,0,127}),
            Line(points={{-73,4},{-59,40},{-49,40}}, color={0,0,127}),
            Line(points={{-113,0},{-76.0373,-0.0180176}}, color={0,0,127}),
            Ellipse(
              extent={{-81.0437,4.59255},{-71.0437,-4.90745}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-73,-5},{-60,-40},{-49,-40}}, color={0,0,127}),
            Line(points={{-72,-2},{-60.0698,-12.907},{-49.0698,-12.907}}, color={
                  0,0,127}),
            Polygon(
              points={{-48.8808,-11},{-48.8808,-15},{-44.8808,-13},{-48.8808,-11}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-46,13},{-35,13},{35,-30},{45,-30}}, color={0,0,127}),
            Line(points={{-45,40},{-35,40},{35,0},{44,0}}, color={0,0,127}),
            Line(points={{-45,-40},{-34,-40},{35,30},{44,30}}, color={0,0,127}),
            Polygon(
              points={{-49,42},{-49,38},{-45,40},{-49,42}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Polygon(
              points={{-48.8728,-38.0295},{-48.8728,-42.0295},{-44.8728,-40.0295},
                  {-48.8728,-38.0295}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Polygon(
              points={{-48.9983,14.8801},{-48.9983,10.8801},{-44.9983,12.8801},{-48.9983,
                  14.8801}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Ellipse(
              extent={{69.3052,4.12743},{79.3052,-5.37257}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{80,0},{100,0}}, color={0,0,127}),
            Polygon(
              points={{43.1618,32.3085},{43.1618,28.3085},{47.1618,30.3085},{
                  43.1618,32.3085}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Polygon(
              points={{43.2575,1.80443},{43.2575,-2.19557},{47.2575,-0.195573},{
                  43.2575,1.80443}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Polygon(
              points={{43.8805,-28.1745},{43.8805,-32.1745},{47.8805,-30.1745},{
                  43.8805,-28.1745}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{48,0},{70,0}}, color={0,0,127}),
            Line(points={{47,30},{60,30},{73,3}}, color={0,0,127}),
            Line(points={{49,-30},{60,-30},{74,-4}}, color={0,0,127}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="extract=%extract")}),
      Documentation(info="<html>
<p>Extract signals from the input connector and transfer them
to the output connector.</p>
<p>The extracting scheme is given by the integer vector 'extract'.
This vector specifies, which input signals are taken and in which
order they are transferred to the output vector. Note, that the
dimension of 'extract' has to match the number of outputs.
Additionally, the dimensions of the input connector signals and
the output connector signals have to be explicitly defined via the
parameters 'nin' and 'nout'.</p>
<p>Example:</p>
<blockquote><pre>
nin  = 7 \"Number of inputs\";
nout = 4 \"Number of outputs\";
extract[nout] = {6,3,3,2} \"Extracting vector\";
</pre></blockquote>
<p>extracts four output signals (nout=4) from the seven elements of the
input vector (nin=7):</p>
<blockquote><pre>
output no. 1 is set equal to input no. 6
output no. 2 is set equal to input no. 3
output no. 3 is set equal to input no. 3
output no. 4 is set equal to input no. 2
</pre></blockquote>
</html>"));
  end ExtractSignal;

  block Extractor
      "Extract scalar signal out of signal vector dependent on IntegerRealInput index"

    extends Blocks.Interfaces.MISO;

    parameter Boolean allowOutOfRange=false "Index may be out of range";
    parameter Real outOfRangeValue=1e10 "Output signal if index is out of range";

    Blocks.Interfaces.IntegerInput index annotation (Placement(transformation(
            origin={0,-120},
            extent={{-20,-20},{20,20}},
            rotation=90)));
    protected
    Real k[nin];
  equation

    when {initial(),change(index)} then

      for i in 1:nin loop
        k[i] = if index == i then 1 else 0;

      end for;

    end when;

    y = if not allowOutOfRange or index > 0 and index <= nin then
                k*u else outOfRangeValue;
    annotation (Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-80,50},{-40,-50}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-84.4104,1.9079},{-84.4104,-2.09208},{-80.4104,-0.09208},{
                  -84.4104,1.9079}},
              lineColor={0,0,127},
              fillPattern=FillPattern.Solid),
            Line(points={{-62,2},{-50.1395,12.907},{-39.1395,12.907}}, color={0,0,127}),
            Line(points={{-63,4},{-49,40},{-39,40}}, color={0,0,127}),
            Line(points={{-102,0},{-65.0373,-0.01802}}, color={0,0,127}),
            Ellipse(
              extent={{-70.0437,4.5925},{-60.0437,-4.90745}},
              lineColor={0,0,127},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid),
            Line(points={{-63,-5},{-50,-40},{-39,-40}}, color={0,0,127}),
            Line(points={{-62,-2},{-50.0698,-12.907},{-39.0698,-12.907}}, color={
                  0,0,127}),
            Polygon(
              points={{-38.8808,-11},{-38.8808,-15},{-34.8808,-13},{-38.8808,-11}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-39,42},{-39,38},{-35,40},{-39,42}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-38.8728,-38.0295},{-38.8728,-42.0295},{-34.8728,-40.0295},
                  {-38.8728,-38.0295}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-38.9983,14.8801},{-38.9983,10.8801},{-34.9983,12.8801},{-38.9983,
                  14.8801}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-30,50},{30,-50}},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{100,0},{0,0}}, color={0,0,127}),
            Line(points={{0,2},{0,-104}}, color={255,128,0}),
            Line(points={{-35,40},{-20,40}}, color={0,0,127}),
            Line(points={{-35,13},{-20,13}}, color={0,0,127}),
            Line(points={{-35,-13},{-20,-13}}, color={0,0,127}),
            Line(points={{-35,-40},{-20,-40}}, color={0,0,127}),
            Polygon(points={{0,0},{-20,13},{-20,13},{0,0},{0,0}}, lineColor={0,0,
                  127}),
            Ellipse(
              extent={{-6,6},{6,-6}},
              lineColor={255,128,0},
              fillColor={255,128,0},
              fillPattern=FillPattern.Solid)}),
                              Documentation(info="<html>
<p>This block extracts a scalar output signal out the
vector of input signals dependent on the Integer
value of the additional u index:</p>
<blockquote><pre>
y = u [ index ] ;
</pre></blockquote>
<p>where index is an additional Integer input signal.</p>
</html>"));
  end Extractor;

    block Multiplex "Multiplexer block for arbitrary number of input connectors"
      extends Blocks.Icons.Block;
      parameter Integer n(min=0)=0 "Dimension of input signal connector" annotation(Dialog(connectorSizing=true), HideResult=true);
      Blocks.Interfaces.RealVectorInput u[n] "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-120,70},{-80,-70}})));
      Blocks.Interfaces.RealOutput y[n + 0] "Connector of Real output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
        y = u;
      annotation (
        defaultComponentName="mux",
        Documentation(info="<html>
<p>
The output connector is the <strong>concatenation</strong> of the input connectors.
</p>
</html>"),
        Icon(
          coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
          graphics={
            Line(points={{8,0},{102,0}}, color={0,0,127}),
            Line(points={{-100,70},{-60,70},{-4,4}}, color={0,0,127}),
            Line(points={{-100,0},{-12,0}}, color={0,0,127}),
            Line(points={{-100,-70},{-60,-70},{-4,-4}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-100,0},{-6,0}}, color={0,0,127}),
            Text(
              extent={{-140,-90},{150,-50}},
              textString="n=%n")}));
    end Multiplex;

    block Multiplex2 "Multiplexer block for two input connectors"
      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of input signal connector 1";
      parameter Integer n2=1 "Dimension of input signal connector 2";
      Blocks.Interfaces.RealInput u1[n1] "Connector of Real input signals 1"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Blocks.Interfaces.RealInput u2[n2] "Connector of Real input signals 2"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Blocks.Interfaces.RealOutput y[n1 + n2]
        "Connector of Real output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      [y] = [u1; u2];
      annotation (
        Documentation(info="<html>
<p>
The output connector is the <strong>concatenation</strong> of the two input connectors.
Note, that the dimensions of the input connector signals have to be
explicitly defined via parameters n1 and n2.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{8,0},{102,0}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-100,60},{-60,60},{0,0}}, color={0,0,127}),
            Line(points={{-100,-60},{-60,-60},{0,0}}, color={0,0,127})}));
    end Multiplex2;

    block Multiplex3 "Multiplexer block for three input connectors"
      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of input signal connector 1";
      parameter Integer n2=1 "Dimension of input signal connector 2";
      parameter Integer n3=1 "Dimension of input signal connector 3";
      Blocks.Interfaces.RealInput u1[n1] "Connector of Real input signals 1"
        annotation (Placement(transformation(extent={{-140,50},{-100,90}})));
      Blocks.Interfaces.RealInput u2[n2] "Connector of Real input signals 2"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealInput u3[n3] "Connector of Real input signals 3"
        annotation (Placement(transformation(extent={{-140,-90},{-100,-50}})));
      Blocks.Interfaces.RealOutput y[n1 + n2 + n3]
        "Connector of Real output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      [y] = [u1; u2; u3];
      annotation (
        Documentation(info="<html>
<p>
The output connector is the <strong>concatenation</strong> of the three input connectors.
Note, that the dimensions of the input connector signals have to be
explicitly defined via parameters n1, n2 and n3.</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{8,0},{102,0}}, color={0,0,127}),
            Line(points={{-100,70},{-60,70},{0,0}}, color={0,0,127}),
            Line(points={{-100,0},{-12,0}}, color={0,0,127}),
            Line(points={{-100,-70},{-60,-70},{0,0}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127})}));
    end Multiplex3;

    block Multiplex4 "Multiplexer block for four input connectors"
      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of input signal connector 1";
      parameter Integer n2=1 "Dimension of input signal connector 2";
      parameter Integer n3=1 "Dimension of input signal connector 3";
      parameter Integer n4=1 "Dimension of input signal connector 4";
      Blocks.Interfaces.RealInput u1[n1] "Connector of Real input signals 1"
        annotation (Placement(transformation(extent={{-140,70},{-100,110}})));
      Blocks.Interfaces.RealInput u2[n2] "Connector of Real input signals 2"
        annotation (Placement(transformation(extent={{-140,10},{-100,50}})));
      Blocks.Interfaces.RealInput u3[n3] "Connector of Real input signals 3"
        annotation (Placement(transformation(extent={{-140,-50},{-100,-10}})));
      Blocks.Interfaces.RealInput u4[n4] "Connector of Real input signals 4"
        annotation (Placement(transformation(extent={{-140,-110},{-100,-70}})));
      Blocks.Interfaces.RealOutput y[n1 + n2 + n3 + n4]
        "Connector of Real output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      [y] = [u1; u2; u3; u4];
      annotation (
        Documentation(info="<html>
<p>
The output connector is the <strong>concatenation</strong> of the four input connectors.
Note, that the dimensions of the input connector signals have to be
explicitly defined via parameters n1, n2, n3 and n4.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{8,0},{102,0}}, color={0,0,127}),
            Line(points={{-100,90},{-60,90},{-3,4}}, color={0,0,127}),
            Line(points={{-100,30},{-60,30},{0,0}}, color={0,0,127}),
            Line(points={{-100,-30},{-60,-30},{0,0}}, color={0,0,127}),
            Line(points={{-100,-90},{-60,-90},{-5,-6}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127})}));
    end Multiplex4;

    block Multiplex5 "Multiplexer block for five input connectors"
      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of input signal connector 1";
      parameter Integer n2=1 "Dimension of input signal connector 2";
      parameter Integer n3=1 "Dimension of input signal connector 3";
      parameter Integer n4=1 "Dimension of input signal connector 4";
      parameter Integer n5=1 "Dimension of input signal connector 5";
      Blocks.Interfaces.RealInput u1[n1] "Connector of Real input signals 1"
        annotation (Placement(transformation(extent={{-140,80},{-100,120}})));
      Blocks.Interfaces.RealInput u2[n2] "Connector of Real input signals 2"
        annotation (Placement(transformation(extent={{-140,30},{-100,70}})));
      Blocks.Interfaces.RealInput u3[n3] "Connector of Real input signals 3"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealInput u4[n4] "Connector of Real input signals 4"
        annotation (Placement(transformation(extent={{-140,-70},{-100,-30}})));
      Blocks.Interfaces.RealInput u5[n5] "Connector of Real input signals 5"
        annotation (Placement(transformation(extent={{-140,-120},{-100,-80}})));
      Blocks.Interfaces.RealOutput y[n1 + n2 + n3 + n4 + n5]
        "Connector of Real output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      [y] = [u1; u2; u3; u4; u5];
      annotation (
        Documentation(info="<html>
<p>
The output connector is the <strong>concatenation</strong> of the five input connectors.
Note, that the dimensions of the input connector signals have to be
explicitly defined via parameters n1, n2, n3, n4 and n5.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{8,0},{102,0}}, color={0,0,127}),
            Line(points={{-100,100},{-60,100},{0,0}}, color={0,0,127}),
            Line(points={{-100,50},{-60,50},{-4,0}}, color={0,0,127}),
            Line(points={{-100,0},{-7,0}}, color={0,0,127}),
            Line(points={{-100,-50},{-60,-50},{-4,0}}, color={0,0,127}),
            Line(points={{-100,-100},{-60,-100},{0,0}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127})}));
    end Multiplex5;

    block Multiplex6 "Multiplexer block for six input connectors"
      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of input signal connector 1";
      parameter Integer n2=1 "Dimension of input signal connector 2";
      parameter Integer n3=1 "Dimension of input signal connector 3";
      parameter Integer n4=1 "Dimension of input signal connector 4";
      parameter Integer n5=1 "Dimension of input signal connector 5";
      parameter Integer n6=1 "Dimension of input signal connector 6";
      Blocks.Interfaces.RealInput u1[n1] "Connector of Real input signals 1"
        annotation (Placement(transformation(extent={{-124,73},{-100,97}})));
      Blocks.Interfaces.RealInput u2[n2] "Connector of Real input signals 2"
        annotation (Placement(transformation(extent={{-124,39},{-100,63}})));
      Blocks.Interfaces.RealInput u3[n3] "Connector of Real input signals 3"
        annotation (Placement(transformation(extent={{-124,5},{-100,29}})));

      Blocks.Interfaces.RealInput u4[n4] "Connector of Real input signals 4"
        annotation (Placement(transformation(extent={{-124,-29},{-100,-5}})));
      Blocks.Interfaces.RealInput u5[n5] "Connector of Real input signals 5"
        annotation (Placement(transformation(extent={{-124,-63},{-100,-39}})));
      Blocks.Interfaces.RealInput u6[n6] "Connector of Real input signals 6"
        annotation (Placement(transformation(extent={{-124,-97},{-100,-73}})));
      Blocks.Interfaces.RealOutput y[n1 + n2 + n3 + n4 + n5 + n6]
        "Connector of Real output signals"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    equation
      [y] = [u1; u2; u3; u4; u5; u6];
      annotation (
        Documentation(info="<html>
<p>
The output connector is the <strong>concatenation</strong> of the six input connectors.
Note, that the dimensions of the input connector signals have to be
explicitly defined via parameters n1, n2, n3, n4, n5 and n6.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{8,0},{102,0}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-100,85},{-60,85},{-3,10}}, color={0,0,127}),
            Line(points={{-100,51},{-60,51},{-7,6}}, color={0,0,127}),
            Line(points={{-100,-17},{-60,-17},{-10,-2}}, color={0,0,127}),
            Line(points={{-100,17},{-60,17},{-10,2}}, color={0,0,127}),
            Line(points={{-100,-51},{-60,-51},{-7,-6}}, color={0,0,127}),
            Line(points={{-100,-85},{-60,-85},{-3,-10}}, color={0,0,127})}));
    end Multiplex6;

    block DeMultiplex "DeMultiplexer block for arbitrary number of output connectors"
      extends Blocks.Icons.Block;
      parameter Integer n(min=0)=0 "Dimension of output signal connector" annotation(Dialog(connectorSizing=true), HideResult=true);
      Blocks.Interfaces.RealInput u[n + 0] "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealVectorOutput y[n]
        "Connector of Real output signals"
        annotation (Placement(transformation(extent={{80,70},{120,-70}})));

    equation
        y = u;
      annotation (
        defaultComponentName="demux",
        Documentation(info="<html>
<p>
The input connector is <strong>split</strong> up into output connectors.
</p>
</html>"),
        Icon(
          coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
          graphics={
            Line(points={{8,0},{102,0}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-100,0},{-6,0}}, color={0,0,127}),
            Line(points={{100,70},{60,70},{4,4}}, color={0,0,127}),
            Line(points={{0,0},{100,0}}, color={0,0,127}),
            Line(points={{100,-70},{60,-70},{4,-4}}, color={0,0,127}),
            Text(
              extent={{-140,-90},{150,-50}},
              textString="n=%n")}));
    end DeMultiplex;

    block DeMultiplex2 "DeMultiplexer block for two output connectors"
      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of output signal connector 1";
      parameter Integer n2=1 "Dimension of output signal connector 2";
      Blocks.Interfaces.RealInput u[n1 + n2] "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y1[n1] "Connector of Real output signals 1"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
      Blocks.Interfaces.RealOutput y2[n2] "Connector of Real output signals 2"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));

    equation
      [u] = [y1; y2];
      annotation (
        Documentation(info="<html>
<p>
The input connector is <strong>split</strong> up into two output connectors.
Note, that the dimensions of the output connector signals have to be
explicitly defined via parameters n1 and n2.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
          graphics={
            Line(points={{100,60},{60,60},{0,0}}, color={0,0,127}),
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{100,-60},{60,-60},{0,0}}, color={0,0,127}),
            Line(points={{-100,0},{-6,0}}, color={0,0,127})}));
    end DeMultiplex2;

    block DeMultiplex3 "DeMultiplexer block for three output connectors"
      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of output signal connector 1";
      parameter Integer n2=1 "Dimension of output signal connector 2";
      parameter Integer n3=1 "Dimension of output signal connector 3";
      Blocks.Interfaces.RealInput u[n1 + n2 + n3]
        "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y1[n1] "Connector of Real output signals 1"
        annotation (Placement(transformation(extent={{100,60},{120,80}})));
      Blocks.Interfaces.RealOutput y2[n2] "Connector of Real output signals 2"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Blocks.Interfaces.RealOutput y3[n3] "Connector of Real output signals 3"
        annotation (Placement(transformation(extent={{100,-80},{120,-60}})));

    equation
      [u] = [y1; y2; y3];
      annotation (
        Documentation(info="<html>
<p>
The input connector is <strong>split</strong> into three output connectors.
Note, that the dimensions of the output connector signals have to be
explicitly defined via parameters n1, n2 and n3.
</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-100,0},{-6,0}}, color={0,0,127}),
            Line(points={{100,70},{60,70},{0,0}}, color={0,0,127}),
            Line(points={{0,0},{100,0}}, color={0,0,127}),
            Line(points={{100,-70},{60,-70},{0,0}}, color={0,0,127})}));
    end DeMultiplex3;

    block DeMultiplex4 "DeMultiplexer block for four output connectors"

      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of output signal connector 1";
      parameter Integer n2=1 "Dimension of output signal connector 2";
      parameter Integer n3=1 "Dimension of output signal connector 3";
      parameter Integer n4=1 "Dimension of output signal connector 4";
      Blocks.Interfaces.RealInput u[n1 + n2 + n3 + n4]
        "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y1[n1] "Connector of Real output signals 1"
        annotation (Placement(transformation(extent={{100,80},{120,100}})));
      Blocks.Interfaces.RealOutput y2[n2] "Connector of Real output signals 2"
        annotation (Placement(transformation(extent={{100,20},{120,40}})));
      Blocks.Interfaces.RealOutput y3[n3] "Connector of Real output signals 3"
        annotation (Placement(transformation(extent={{100,-40},{120,-20}})));
      Blocks.Interfaces.RealOutput y4[n4] "Connector of Real output signals 4"
        annotation (Placement(transformation(extent={{100,-100},{120,-80}})));

    equation
      [u] = [y1; y2; y3; y4];
      annotation (
        Documentation(info="<html>
<p>
The input connector is <strong>split</strong> into four output connectors.
Note, that the dimensions of the output connector signals have to be
explicitly defined via parameters n1, n2, n3 and n4.</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-100,0},{-6,0}}, color={0,0,127}),
            Line(points={{100,90},{60,90},{0,0}}, color={0,0,127}),
            Line(points={{100,30},{60,30},{0,0}}, color={0,0,127}),
            Line(points={{100,-30},{60,-30},{0,0}}, color={0,0,127}),
            Line(points={{100,-90},{60,-90},{0,0}}, color={0,0,127})}));
    end DeMultiplex4;

    block DeMultiplex5 "DeMultiplexer block for five output connectors"

      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of output signal connector 1";
      parameter Integer n2=1 "Dimension of output signal connector 2";
      parameter Integer n3=1 "Dimension of output signal connector 3";
      parameter Integer n4=1 "Dimension of output signal connector 4";
      parameter Integer n5=1 "Dimension of output signal connector 5";
      Blocks.Interfaces.RealInput u[n1 + n2 + n3 + n4 + n5]
        "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y1[n1] "Connector of Real output signals 1"
        annotation (Placement(transformation(extent={{100,70},{120,90}})));
      Blocks.Interfaces.RealOutput y2[n2] "Connector of Real output signals 2"
        annotation (Placement(transformation(extent={{100,30},{120,50}})));
      Blocks.Interfaces.RealOutput y3[n3] "Connector of Real output signals 3"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Blocks.Interfaces.RealOutput y4[n4] "Connector of Real output signals 4"
        annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
      Blocks.Interfaces.RealOutput y5[n5] "Connector of Real output signals 5"
        annotation (Placement(transformation(extent={{100,-90},{120,-70}})));

    equation
      [u] = [y1; y2; y3; y4; y5];
      annotation (
        Documentation(info="<html>
<p>
The input connector is <strong>split</strong> into five output connectors.
Note, that the dimensions of the output connector signals have to be
explicitly defined via parameters n1, n2, n3, n4 and n5.</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-100,0},{-6,0}}, color={0,0,127}),
            Line(points={{100,80},{60,80},{0,0}}, color={0,0,127}),
            Line(points={{100,40},{60,40},{8,4}}, color={0,0,127}),
            Line(points={{100,0},{10,0}}, color={0,0,127}),
            Line(points={{100,-40},{60,-40},{8,-4}}, color={0,0,127}),
            Line(points={{100,-80},{60,-80},{0,0}}, color={0,0,127})}));
    end DeMultiplex5;

    block DeMultiplex6 "DeMultiplexer block for six output connectors"
      extends Blocks.Icons.Block;
      parameter Integer n1=1 "Dimension of output signal connector 1";
      parameter Integer n2=1 "Dimension of output signal connector 2";
      parameter Integer n3=1 "Dimension of output signal connector 3";
      parameter Integer n4=1 "Dimension of output signal connector 4";
      parameter Integer n5=1 "Dimension of output signal connector 5";
      parameter Integer n6=1 "Dimension of output signal connector 6";
      Blocks.Interfaces.RealInput u[n1 + n2 + n3 + n4 + n5 + n6]
        "Connector of Real input signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.RealOutput y1[n1] "Connector of Real output signals 1"
        annotation (Placement(transformation(extent={{100,80},{120,100}})));
      Blocks.Interfaces.RealOutput y2[n2] "Connector of Real output signals 2"
        annotation (Placement(transformation(extent={{100,44},{120,64}})));
      Blocks.Interfaces.RealOutput y3[n3] "Connector of Real output signals 3"
        annotation (Placement(transformation(extent={{100,8},{120,28}})));
      Blocks.Interfaces.RealOutput y4[n4] "Connector of Real output signals 4"
        annotation (Placement(transformation(extent={{100,-28},{120,-8}})));
      Blocks.Interfaces.RealOutput y5[n5] "Connector of Real output signals 5"
        annotation (Placement(transformation(extent={{100,-64},{120,-44}})));
      Blocks.Interfaces.RealOutput y6[n6] "Connector of Real output signals 6"
        annotation (Placement(transformation(extent={{100,-100},{120,-80}})));

    equation
      [u] = [y1; y2; y3; y4; y5; y6];
      annotation (
        Documentation(info="<html>
<p>
The input connector is <strong>split</strong> into six output connectors.
Note, that the dimensions of the output connector signals have to be
explicitly defined via parameters n1, n2, n3, n4, n5 and n6.</p>
</html>"),   Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(
              extent={{-15,15},{15,-15}},
              fillColor={0,0,127},
              fillPattern=FillPattern.Solid,
              lineColor={0,0,127}),
            Line(points={{-100,0},{-6,0}}, color={0,0,127}),
            Line(points={{100,90},{60,90},{0,4}}, color={0,0,127}),
            Line(points={{100,54},{60,54},{8,6}}, color={0,0,127}),
            Line(points={{100,18},{60,18},{10,2}}, color={0,0,127}),
            Line(points={{100,-18},{60,-18},{10,-2}}, color={0,0,127}),
            Line(points={{100,-54},{60,-54},{8,-6}}, color={0,0,127}),
            Line(points={{100,-90},{60,-90},{0,-4}}, color={0,0,127})}));
    end DeMultiplex6;

    model RealPassThrough "Pass a Real signal through without modification"
      extends Blocks.Interfaces.SISO;
    equation
      y = u;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={Line(points={{-100,0},{100,0}},
                color={0,0,127})}),
                        Documentation(info="<html>
<p>
Passes a Real signal through without modification.  Enables signals to be read out of one bus, have their name changed and be sent back to a bus.
</p>
</html>"));
    end RealPassThrough;

    model IntegerPassThrough "Pass a Integer signal through without modification"
      extends Blocks.Icons.IntegerBlock;

      Blocks.Interfaces.IntegerInput u "Input signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Blocks.Interfaces.IntegerOutput y "Output signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    equation
      y = u;

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={Line(points={{-100,0},{100,0}},
                color={255,128,0})}),
                        Documentation(info="<html>
<p>Passes a Integer signal through without modification.  Enables signals to be read out of one bus, have their name changed and be sent back to a bus.</p>
</html>"));
    end IntegerPassThrough;

    model BooleanPassThrough "Pass a Boolean signal through without modification"
      extends Blocks.Interfaces.BooleanSISO;
    equation
      y = u;
      annotation (Documentation(info="<html>
<p>Passes a Boolean signal through without modification.  Enables signals to be read out of one bus, have their name changed and be sent back to a bus.</p>
</html>"),
        Icon(
          coordinateSystem(preserveAspectRatio=true,
              extent={{-100.0,-100.0},{100.0,100.0}}),
              graphics={
          Line(
            points={{-100.0,0.0},{100.0,0.0}},
            color={255,0,255})}));
    end BooleanPassThrough;
    annotation (Documentation(info="<html>
<p>
This package contains blocks to combine and extract signals.
</p>
</html>"),   Icon(graphics={
          Line(points={{-90,0},{4,0}}, color={95,95,95}),
          Line(points={{88,65},{48,65},{-8,0}}, color={95,95,95}),
          Line(points={{-8,0},{93,0}}, color={95,95,95}),
          Line(points={{87,-65},{48,-65},{-8,0}}, color={95,95,95})}));
  end Routing;

  package Noise "Library of noise blocks"
    extends Modelica.Icons.Package;

    model GlobalSeed
      "Defines global settings for the blocks of sublibrary Noise, especially a global seed value is defined"
      parameter Boolean enableNoise = true
        "= true, if noise blocks generate noise as output; = false, if they generate a constant output"
        annotation(choices(checkBox=true));
      parameter Boolean useAutomaticSeed = false
        "= true, choose a seed by system time and process id; = false, use fixedSeed"
        annotation(choices(checkBox=true));
      parameter Integer fixedSeed = 67867967
        "Fixed global seed for random number generators (if useAutomaticSeed = false)"
          annotation(Dialog(enable=not useAutomaticSeed));
      final parameter Integer seed(fixed=false) "Actually used global seed";
      final parameter Integer id_impure(fixed=false)
        "ID for impure random number generators Modelica.Math.Random.Utilities.impureXXX" annotation(HideResult=true);
    initial equation
      seed = if useAutomaticSeed then Modelica.Math.Random.Utilities.automaticGlobalSeed() else fixedSeed;
      id_impure = Modelica.Math.Random.Utilities.initializeImpureRandom(seed);

      annotation (
        defaultComponentName="globalSeed",
        defaultComponentPrefixes="inner",
        missingInnerMessage="
Your model is using an outer \"globalSeed\" component but
an inner \"globalSeed\" component is not defined and therefore
a default inner \"globalSeed\" component is introduced by the tool.
To change the default setting, drag Noise.GlobalSeed
into your model and specify the seed.
",   Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                             graphics={Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
                                            Text(
            extent={{-150,150},{150,110}},
            textString="%name",
            textColor={0,0,255}),
            Line(visible = enableNoise,
                 points={{-73,-15},{-59,-15},{-59,1},{-51,1},{-51,-47},{-43,-47},{-43,
                  -25},{-35,-25},{-35,59},{-27,59},{-27,27},{-27,27},{-27,-33},{-17,-33},{-17,-15},{-7,-15},{-7,-43},{3,
                  -43},{3,39},{9,39},{9,53},{15,53},{15,-3},{25,-3},{25,9},{31,9},{31,
                  -21},{41,-21},{41,51},{51,51},{51,17},{59,17},{59,-49},{69,-49}},
                color={215,215,215}),
            Text(visible=enableNoise and not useAutomaticSeed,
              extent={{-90,-4},{88,-30}},
              textColor={255,0,0},
              textString="%fixedSeed"),
            Line(visible = not enableNoise,
              points={{-80,-4},{84,-4}},
              color={215,215,215}),
            Text(visible=enableNoise and not useAutomaticSeed,
              extent={{-84,34},{94,8}},
              textColor={255,0,0},
              textString="fixedSeed =")}),
        Documentation(revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>",     info="<html>
<p>
When using one of the blocks of sublibrary <a href=\"modelica://Modelica.Blocks.Noise\">Noise</a>,
on the same or a higher hierarchical level, Noise.GlobalSeed
must be dragged resulting in a declaration
</p>

<blockquote><pre>
<strong>inner</strong> Modelica.Blocks.Noise.GlobalSeed globalSeed;
</pre></blockquote>

<p>
The GlobalSeed block provides global options for all Noise blocks of the same or a lower
hierarchical level. The following options can be selected:
</p>

<blockquote>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Icon</th>
    <th>Description</th></tr>

<tr><td> <img src=\"modelica://Modelica/Resources/Images/Blocks/Noise/GlobalSeed_FixedSeed.png\"> </td>
    <td> <strong>useAutomaticSeed=false</strong> (= default):<br>
         A fixed global seed is defined with Integer parameter fixedSeed. The value of fixedSeed
         is displayed in the icon. By default all Noise blocks use fixedSeed for initialization of their
         pseudo random number generators, in combination with a local seed defined for every instance
         separately. Therefore, whenever a simulation is performed with the
         same fixedSeed exactly the same noise is generated in all instances of the Noise
         blocks (provided the settings of these blocks are not changed as well).<br>
         This option can be used (a) to design a control system (e.g. by parameter optimization) and keep the same
         noise for all simulations, or (b) perform Monte Carlo Simulations where
         fixedSeed is changed from the environment for every simulation, in order to
         produce different noise at every simulation run.</td></tr>

<tr><td> <img src=\"modelica://Modelica/Resources/Images/Blocks/Noise/GlobalSeed_AutomaticSeed.png\"> </td>
    <td> <strong>useAutomaticSeed=true</strong>:<br>
         An automatic global seed is computed by using the ID of the process in which the
         simulation takes place and the current local time. As a result, the global seed
         is changed automatically for every new simulation, including parallelized
         simulation runs. This option can be used to perform Monte Carlo Simulations
         with minimal effort (just performing many simulation runs) where
         every simulation run uses a different noise.</td></tr>

<tr><td> <img src=\"modelica://Modelica/Resources/Images/Blocks/Noise/GlobalSeed_NoNoise.png\"> </td>
    <td> <strong>enableNoise=false</strong>:<br>
         The noise in all Noise instances is switched off and the blocks output a constant
         signal all the time (usually zero). This option is useful, if a model shall be
         tested without noise and the noise shall be quickly turned off or on.</td></tr>
</table>
</blockquote>

<p>
Additionally, the globalSeed instance calls function
<a href=\"modelica://Modelica.Math.Random.Utilities.initializeImpureRandom\">initializeImpureRandom</a>
to initialize the impure random number generators
(<a href=\"modelica://Modelica.Math.Random.Utilities.impureRandom\">impureRandom</a> and
<a href=\"modelica://Modelica.Math.Random.Utilities.impureRandomInteger\">impureRandomInteger</a>).
The return value of this function is stored in parameter <strong>id_impure</strong>. Whenever one of the impure
random number generators need to be called, \"globalSeed.id_impure\" has to be given as input argument.
</p>

<p>
Note, the usage of this block is demonstrated with examples
<a href=\"modelica://Modelica.Blocks.Examples.Noise.AutomaticSeed\">AutomaticSeed</a> and
<a href=\"modelica://Modelica.Blocks.Examples.Noise.ImpureGenerator\">ImpureGenerator</a>.
</p>

<p>
Please note that only one globalSeed instance may be defined in the model due to the initialization
of the impure random number generators with <a href=\"modelica://Modelica.Math.Random.Utilities.initializeImpureRandom\">initializeImpureRandom</a>!
So, the block will usually reside on the top level of the model.
</p>
</html>"));
    end GlobalSeed;

    block UniformNoise "Noise generator with uniform distribution"
      import distribution = Modelica.Math.Distributions.Uniform.quantile;
      extends Blocks.Interfaces.PartialNoise;

      // Main dialog menu
      parameter Real y_min(start=0) "Lower limit of y" annotation(Dialog(enable=enableNoise));
      parameter Real y_max(start=1) "Upper limit of y" annotation(Dialog(enable=enableNoise));

    initial equation
       r = distribution(r_raw, y_min, y_max);

    equation
      // Draw random number at sample times
      when generateNoise and sample(startTime, samplePeriod) then
        r = distribution(r_raw, y_min, y_max);
      end when;

        annotation(Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics={
            Line(visible=enableNoise,
              points={{-76,60},{78,60}}, color={95,95,95},
              pattern=LinePattern.Dot),
            Line(visible=enableNoise,
              points={{-76,-60},{78,-60}}, color={95,95,95},
              pattern=LinePattern.Dot),
            Text(visible=enableNoise,
              extent={{-70,94},{95,64}},
              textColor={175,175,175},
              textString="%y_max"),
            Text(visible=enableNoise,
              extent={{-70,-64},{95,-94}},
              textColor={175,175,175},
              textString="%y_min")}),
        Documentation(info="<html>
<p>
A summary of the common properties of the noise blocks is provided in the documentation of package
<a href=\"modelica://Modelica.Blocks.Noise\">Blocks.Noise</a>.
This UniformNoise block generates reproducible, random noise at its output according to a uniform distribution.
This means that random values are uniformly distributed within the range defined by parameters
y_min and y_max (see example <a href=\"modelica://Modelica.Blocks.Examples.Noise.UniformNoiseProperties\">Noise.UniformNoiseProperties</a>).
By default, two or more instances produce different, uncorrelated noise at the same time instant.
The block can only be used if on the same or a higher hierarchical level,
model <a href=\"modelica://Modelica.Blocks.Noise.GlobalSeed\">Blocks.Noise.GlobalSeed</a>
is dragged to provide global settings for all instances.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end UniformNoise;

    block NormalNoise "Noise generator with normal distribution"
      import distribution = Modelica.Math.Distributions.Normal.quantile;
      extends Blocks.Interfaces.PartialNoise;

      // Main dialog menu
      parameter Real mu=0 "Expectation (mean) value of the normal distribution" annotation(Dialog(enable=enableNoise));
      parameter Real sigma(start=1)
        "Standard deviation of the normal distribution" annotation(Dialog(enable=enableNoise));

    initial equation
       r = distribution(r_raw, mu, sigma);

    equation
      // Draw random number at sample times
      when generateNoise and sample(startTime, samplePeriod) then
        r = distribution(r_raw, mu, sigma);
      end when;

        annotation(Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics={
                Text(visible=enableNoise,
                 extent={{-66,92},{94,66}},
                 textColor={175,175,175},
                 textString="mu=%mu"),
                Text(visible=enableNoise,
                 extent={{-70,-68},{94,-96}},
                 textColor={175,175,175},
                 textString="sigma=%sigma")}),
        Documentation(info="<html>
<p>
A summary of the common properties of the noise blocks is provided in the documentation of package
<a href=\"modelica://Modelica.Blocks.Noise\">Blocks.Noise</a>.
This NormalNoise block generates reproducible, random noise at its output according to a normal distribution.
This means that random values are normally distributed with expectation value mu and standard deviation sigma.
(see example <a href=\"modelica://Modelica.Blocks.Examples.Noise.NormalNoiseProperties\">Examples.Noise.NormalNoiseProperties</a>).
By default, two or more instances produce different, uncorrelated noise at the same time instant.
The block can only be used if on the same or a higher hierarchical level,
model <a href=\"modelica://Modelica.Blocks.Noise.GlobalSeed\">Blocks.Noise.GlobalSeed</a>
is dragged to provide global settings for all instances.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end NormalNoise;

    block TruncatedNormalNoise
      "Noise generator with truncated normal distribution"
      import distribution =
        Modelica.Math.Distributions.TruncatedNormal.quantile;
      extends Blocks.Interfaces.PartialNoise;

      // Main dialog menu
      parameter Real y_min(start=0) "Lower limit of y" annotation(Dialog(enable=enableNoise));
      parameter Real y_max(start=1) "Upper limit of y" annotation(Dialog(enable=enableNoise));
      parameter Real mu =    (y_max + y_min)/2
        "Expectation (mean) value of the normal distribution" annotation(Dialog(enable=enableNoise,tab="Advanced",group="Noise generation"));
      parameter Real sigma = (y_max - y_min)/6
        "Standard deviation of the normal distribution" annotation(Dialog(enable=enableNoise,tab="Advanced",group="Noise generation"));

    initial equation
       r = distribution(r_raw, y_min, y_max, mu, sigma);

    equation
      // Draw random number at sample times
      when generateNoise and sample(startTime, samplePeriod) then
        r = distribution(r_raw, y_min, y_max, mu, sigma);
      end when;

        annotation(Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics={
            Line(visible=enableNoise,
              points={{-76,60},{78,60}}, color={95,95,95},
              pattern=LinePattern.Dot),
            Line(visible=enableNoise,
              points={{-76,-60},{78,-60}}, color={95,95,95},
              pattern=LinePattern.Dot),
            Text(visible=enableNoise,
              extent={{-70,94},{95,64}},
              textColor={175,175,175},
              textString="%y_max"),
            Text(visible=enableNoise,
              extent={{-70,-64},{95,-94}},
              textColor={175,175,175},
              textString="%y_min"),
            Text(
              extent={{-71,12},{71,-12}},
              textColor={175,175,175},
              origin={-88,-11},
              rotation=90,
              textString="normal")}),
        Documentation(info="<html>
<p>
A summary of the common properties of the noise blocks is provided in the documentation of package
<a href=\"modelica://Modelica.Blocks.Noise\">Blocks.Noise</a>.
This TruncatedNormalNoise block generates reproducible, random noise at its output according to a truncated normal distribution.
This means that normally distributed random values are truncated to the band y_min ... y_max.
Measurement noise has often this distribution form.
By default, the standard parameters of the truncated normal distribution are derived from
y_min ... y_max:
</p>
<blockquote><p>
mean value = (y_max + y_min)/2,<br>
standard deviation = (y_max - y_min)/6 (= 99.7 % of the non-truncated normal distribution are within y_min ... y_max).
</p></blockquote>

<p>
For an example see <a href=\"modelica://Modelica.Blocks.Examples.Noise.Distributions\">Examples.Noise.Distributions</a>.
By default, two or more instances produce different, uncorrelated noise at the same time instant.
The block can only be used if on the same or a higher hierarchical level,
model <a href=\"modelica://Modelica.Blocks.Noise.GlobalSeed\">Blocks.Noise.GlobalSeed</a>
is dragged to provide global settings for all instances.
</p>
</html>",     revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
    end TruncatedNormalNoise;

    block BandLimitedWhiteNoise
      "Noise generator to produce band-limited white noise with normal distribution"
      import distribution = Modelica.Math.Distributions.Normal.quantile;
      extends Blocks.Interfaces.PartialNoise;

      // Main dialog menu
      parameter Real noisePower = 1 "Power of white noise signal" annotation(Dialog(enable=enableNoise));

    protected
      parameter Real mu=0;
      parameter Real sigma=sqrt(noisePower/samplePeriod);

    initial equation
       r = distribution(r_raw, mu, sigma);

    equation
      // Draw random number at sample times
      when generateNoise and sample(startTime, samplePeriod) then
        r = distribution(r_raw, mu, sigma);
      end when;
                                                                                          annotation(Dialog(enable=enableNoise), Icon(
            graphics={Text(
              extent={{-70,96},{92,60}},
              textColor={175,175,175},
              textString="%noisePower"),
            Text(
              extent={{-96,11},{96,-11}},
              textColor={175,175,175},
              origin={-87,0},
              rotation=90,
              textString="white noise")}),
                  Documentation(info="<html>
<p>
A summary of the common properties of the noise blocks is provided in the documentation of package
<a href=\"modelica://Modelica.Blocks.Noise\">Blocks.Noise</a>.
This BandLimitedWhiteNoise block generates reproducible, random noise at its output according to a
band-limited white noise distribution. This is performed by using a normal distribution with mu=0 and
sigma = sqrt(noisePower/samplePeriod).
</p>

<p>
In order for this block to produce meaningful results, you should set the following
parameters:
</p>

<ul>
<li> The <strong>samplePeriod</strong> of the block should be much faster (say by a factor of 100)
     than the fastest dynamics of the system fed by the block&#39;s outputs.</li>
<li> The <strong>noisePower</strong> of the signal should be set to the expected power per frequency
     of the white noise. Since many system models assume a noise power of 1,
     this preset may be a reasonable first choice (= default).</li>
</ul>

<h4>About sampling frequencies</h4>

<p>
Ideal white noise contains all frequencies, including infinitely high ones.
However, these usually cannot be observed in physical systems, since all physical systems in
one way or the other contain low-pass filters. It is thus sufficient to generate a
limited range of frequency content in the noise signal, as long as it exceeds the frequencies of
the subsequent dynamics by a sufficiently high factor (of e.g. 100).
</p>

<h4>About noise power</h4>

<p>
Ideal white noise has a flat, i.e. constant, power spectral density for all frequencies.
It has thus infinitely high power, because the total power of a signal can be obtained by
integrating the power spectral density over all frequencies. The following three ways to
think of the power of a signal may be helpful:
</p>

<ul>
<li> The energy of a signal is the integral of its squared absolute value over time.
     The signal&#39;s power is this integral divided by the time span of the integral.</li>
<li> The total power of a signal can also be obtained by integrating its (two-sided)
     power spectral density over all frequencies.</li>
<li> The total power of a signal is finally also equal to its variance.</li>
</ul>

<p>
In order to set the correct level of the band-limited white noise power spectral density,
the variance of its normal distribution can thus be influenced directly.
Recalling that the samplePeriod of the noise signal generates frequency content in the
range &plusmn;0.5/samplePeriod, the variance must be increased to generate sufficient
total signal power. The total power must match the product of the noisePower and its
frequency bandwidth 1/samplePeriod: <code>signal power = signal variance = noisePower / samplePeriod</code>.
</p>

<p>
Example <a href=\"modelica://Modelica.Blocks.Examples.Noise.DrydenContinuousTurbulence\">Examples.Noise.DrydenContinuousTurbulence</a>
demonstrates how to utilize this block to model wind gust.
</p>
</html>"));
    end BandLimitedWhiteNoise;
    annotation (Icon(graphics={Line(
        points={{-84,0},{-54,0},{-54,40},{-24,40},{-24,-70},{6,-70},{6,80},
            {36,80},{36,-20},{66,-20},{66,60}})}), Documentation(info="<html>
<p>
This sublibrary contains blocks that generate <strong>reproducible noise</strong> with pseudo random
numbers. Reproducibility is important when designing control systems,
either manually or with optimization methods (for example when changing a parameter or a component
of a control system and re-simulating, it is important that the noise does not change, because
otherwise it is hard to determine whether the changed control system or the differently
computed noise has changed the behaviour of the controlled system).
Many examples how to use the Noise blocks are provided in sublibrary
<a href=\"modelica://Modelica.Blocks.Examples.Noise\">Blocks.Examples.Noise</a>.
</p>

<h4>Global Options</h4>

<p>
When using one of the blocks of this sublibrary, on the same or a higher level,
block <a href=\"modelica://Modelica.Blocks.Noise.GlobalSeed\">Noise.GlobalSeed</a>
must be dragged resulting in a declaration
</p>

<blockquote><pre>
<strong>inner</strong> Modelica.Blocks.Noise.GlobalSeed globalSeed;
</pre></blockquote>

<p>
This block is used to define global options that hold for all Noise block
instances (such as a global seed for initializing the random number generators,
and a flag to switch off noise). Furthermore, the impure random number generator
<a href=\"modelica://Modelica.Math.Random.Utilities.impureRandom\">impureRandom</a> is initialized here.
</p>

<p>
Please note that only one globalSeed instance may be defined in the model due to the initialization
of the impureRandom(..) random number generator! So, the block will usually reside on the top level of the model.
</p>

<h4>Parameters that need to be defined</h4>

<p>
When using a noise block of this package, at a minimum the following parameters must be defined:
</p>

<blockquote>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Parameter</th>
    <th>Description</th></tr>

<tr><td> samplePeriod </td>
    <td> Random values are drawn periodically at the sample rate in [s]
         defined with this parameter (time events are generated at the sample instants).
         Between sample instants, the output y is kept constant.</td></tr>

<tr><td> distribution data </td>
    <td> Every noise block in this package needs additional data to describe the respective
         distribution. A random number distribution maps the drawn random numbers
         from the range 0.0 ... 1.0, to the desired range and distribution.
         </td></tr>
</table>
</blockquote>

<p>
As a simple demonstration, see example <a href=\"modelica://Modelica.Blocks.Examples.Noise.UniformNoise\">Blocks.Examples.Noise.UniformNoise</a>.
In the next diagram, a simulation result is shown for samplePeriod=0.02 s and uniform distribution with
y_min=-1, y_max=3:
</p>
<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Examples/Noise/UniformNoise.png\">
</blockquote>

<h4>Advanced tab: General settings</h4>
<p>
In the <strong>Advanced</strong> tab of the parameter menu, further options can be set in the noise blocks
as shown in the next table:
</p>

<blockquote>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Parameter</th>
    <th>Description</th></tr>

<tr><td> enableNoise </td>
    <td> = true, if noise is generated at the output of the block (this is the default).<br>
         = false, if noise generation is switched off and the constant value
         y_off is provided as output.</td></tr>
<tr><td> y_off </td>
    <td> If enableNoise = false, the output of the block instance has the
         value y_off. Default is y_off = 0.0.
         Furthermore, if enableNoise = true and time&lt;startTime, the output of the block is also
         y_off (see description of parameter startTime below).</td></tr>
</table>
</blockquote>

<h4>Advanced tab: Initialization</h4>

<p>
For every block instance, the internally used pseudo random number generator
has its own state. This state must be properly initialized, depending on
the desired situation. For this purpose the following parameters can be defined:
</p>

<blockquote>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Parameter</th>
    <th>Description</th></tr>

<tr><td> useGlobalSeed </td>
    <td> = true, if the seed (= Integer number) defined in the \"inner GlobalSeed globalSeed\"
         component is used for the initialization of the random number generator used in this
         instance of the noise block.
         Therefore, whenever the globalSeed defines a different number, the noise at every
         instance is changing. This is the default setting and therefore the globalSeed component
         defines whether every new simulation run shall provide the same noise
         (e.g. for a parameter optimization of controller parameters), or
         whether every new simulation run shall provide different noise
         (e.g. for a Monte Carlo simulation).<br>
         = false, if the seed defined by globalSeed is ignored. For example, if
         aerodynamic turbulence is modelled with a noise block and this turbulence
         model shall be used for all simulation runs of a Monte Carlo simulation, then
         useGlobalSeed has to be set to false.</td></tr>

<tr><td> useAutomaticLocalSeed </td>
    <td> An Integer number, called local seed, is needed to initialize the random number
         generator for a specific block instance. Instances using the same local seed
         produce exactly the same random number values (so the same noise, if the other settings
         of the instances are the same).<br>
         If <strong>useAutomaticLocalSeed = true</strong>, the
         local seed is determined automatically using a hash value of the instance name of the model that is
         inquired with the Modelica built-in operator <a href=\"https://specification.modelica.org/v3.4/Ch3.html#getinstancename\">getInstanceName()</a>.
         Note, this means that the noise changes if the component is renamed.<br>
         If <strong>useAutomaticLocalSeed = false</strong>, the local seed is defined
         explicitly by parameter fixedLocalSeed. It is then guaranteed that the generated noise
         remains always the same (provided the other parameter values are the same).</td></tr>

<tr><td> fixedLocalSeed </td>
    <td> If useAutomaticLocalSeed = false, the local seed to be used.
         fixedLocalSeed can be any Integer number (including zero or a negative number).
         The initialization algorithm produces a meaningful initial state of the random
         number generator from fixedLocalSeed and (if useAutomaticGlobalSeed=true) from globalSeed even for
         bad seeds such as 0 or 1, so the subsequently drawing of random numbers produces always statistically
         meaningful numbers.</td></tr>

<tr><td> startTime </td>
    <td> The time instant at which noise shall be generated at the output y. The default
         startTime = 0.
         For time&lt;startTime, y = y_off. In some cases it is meaningful to simulate
         a certain duration until an approximate steady-state is reached. In such a case
         startTime should be set to a time instant after this duration.</td></tr>
</table>
</blockquote>

<h4>Random Number Generators</h4>

<p>
The core of the noise generation is the computation of uniform random
numbers in the range 0.0 .. 1.0 (and these random numbers are transformed
afterwards, see below). This sublibrary uses the xorshift random number generation
suite developed in 2014 by Sebastiano Vigna (for details see
<a href=\"http://xorshift.di.unimi.it\">http://xorshift.di.unimi.it</a> and
<a href=\"modelica://Modelica.Math.Random.Generators\">Math.Random.Generators</a>).
These random number generators have excellent
statistical properties, produce quickly statistically relevant random numbers, even if
starting from a bad initial seed, and have a reasonable length of the internal state
vector of 2, 4, and 33 Integer elements. The random number generator with an internal
state vector of length 2 is used to initialize the other two random number generators.
The length 4 random number generator is used in the noise blocks of this package, and every
such block has its own internal state vector, as needed for reproducible noise blocks.
The random number generator with a length of 33 Integer is used from the impure random number
generator. It is suited even for massively parallel simulations where every simulation
computes a large number of random values. More details of the random number
generators are described in the documentation of package
<a href=\"modelica://Modelica.Math.Random.Generators\">Math.Random.Generators</a>.
</p>

<h4>Distributions</h4>

<p>
The uniform random numbers in the range 0.0 .. 1.0 are transformed to a desired
random number distribution by selecting an appropriate <strong>distribution</strong> or
<strong>truncated distribution</strong>. For an example of a truncated distribution, see the following
diagram of the probability density function of a normal distribution
compared with its truncated version:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Math/Distributions/TruncatedNormal.density.png\">
</blockquote>

<p>
The corresponding inverse cumulative distribution functions are shown in the next diagram:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Math/Distributions/TruncatedNormal.quantile.png\">
</blockquote>

<p>
When providing an x-value between 0.0 .. 1.0 from a random number generator, then the truncated
inverse cumulative probability density function of a normal distribution transforms this value into the
desired band (in the diagram above to the range: -1.5 .. 1.5). Contrary to a standard distribution,
truncated distributions have the advantage that the resulting random values are guaranteed
to be in the defined band (whereas a standard normal distribution might also result in any value;
when modeling noise that is known to be in a particular range, say &plusmn; 0.1 Volt,
then with the TruncatedNormal distribution it is guaranteed that random values are only
generated in this band). More details of truncated
distributions are given in the documentation of package
<a href=\"modelica://Modelica.Math.Distributions\">Math.Distributions</a>.
</p>
</html>",   revisions="<html>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
<tr><th>Date</th> <th align=\"left\">Description</th></tr>

<tr><td> June 22, 2015 </td>
    <td>

<table border=\"0\">
<tr><td>
         <img src=\"modelica://Modelica/Resources/Images/Logos/dlr_logo.png\" alt=\"DLR logo\">
</td><td valign=\"bottom\">
         Initial version implemented by
         A. Kl&ouml;ckner, F. v.d. Linden, D. Zimmer, M. Otter.<br>
         <a href=\"http://www.dlr.de/rmc/sr/en\">DLR Institute of System Dynamics and Control</a>
</td></tr></table>
</td></tr>

</table>
</html>"));
  end Noise;

  package Sources
    "Library of signal source blocks generating Real, Integer and Boolean signals"
    import Blocks.Interfaces;

    extends Modelica.Icons.SourcesPackage;

    block RealExpression "Set output signal to a time varying Real expression"

      Blocks.Interfaces.RealOutput y=0.0 "Value of Real output" annotation (
          Dialog(group="Time varying output signal"), Placement(transformation(
              extent={{100,-10},{120,10}})));

      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-100,40},{100,-40}},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised),
            Text(
              extent={{-96,15},{96,-15}},
              textString="%y"),
            Text(
              extent={{-150,90},{150,50}},
              textString="%name",
              textColor={0,0,255})}), Documentation(info="<html>
<p>
The (time varying) Real output signal of this block can be defined in its
parameter menu via variable <strong>y</strong>. The purpose is to support the
easy definition of Real expressions in a block diagram. For example,
in the y-menu the definition \"if time &lt; 1 then 0 else 1\" can be given in order
to define that the output signal is one, if time &ge; 1 and otherwise
it is zero. Note, that \"time\" is a built-in variable that is always
accessible and represents the \"model time\" and that
variable <strong>y</strong> is both a variable and a connector.
</p>
</html>"));

    end RealExpression;

    block IntegerExpression
      "Set output signal to a time varying Integer expression"

      Blocks.Interfaces.IntegerOutput y=0 "Value of Integer output" annotation
        (Dialog(group="Time varying output signal"), Placement(transformation(
              extent={{100,-10},{120,10}})));

      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-100,40},{100,-40}},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised),
            Text(
              extent={{-96,15},{96,-15}},
              textString="%y"),
            Text(
              extent={{-150,90},{150,50}},
              textString="%name",
              textColor={0,0,255})}), Documentation(info="<html>
<p>
The (time varying) Integer output signal of this block can be defined in its
parameter menu via variable <strong>y</strong>. The purpose is to support the
easy definition of Integer expressions in a block diagram. For example,
in the y-menu the definition \"if time &lt; 1 then 0 else 1\" can be given in order
to define that the output signal is one, if time &ge; 1 and otherwise
it is zero. Note, that \"time\" is a built-in variable that is always
accessible and represents the \"model time\" and that
variable <strong>y</strong> is both a variable and a connector.
</p>
</html>"));

    end IntegerExpression;

    block BooleanExpression
      "Set output signal to a time varying Boolean expression"

      Blocks.Interfaces.BooleanOutput y=false "Value of Boolean output"
        annotation (Dialog(group="Time varying output signal"), Placement(
            transformation(extent={{100,-10},{120,10}})));

      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}}), graphics={
            Rectangle(
              extent={{-100,40},{100,-40}},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised),
            Text(
              extent={{-96,15},{96,-15}},
              textString="%y"),
            Text(
              extent={{-150,90},{150,50}},
              textString="%name",
              textColor={0,0,255}),
            Polygon(
              points={{100,10},{120,0},{100,-10},{100,10}},
              lineColor=DynamicSelect({255,0,255}, if y then {0,255,0} else {255,0,255}),
              fillColor=DynamicSelect({255,255,255}, if y then {0,255,0} else {255,255,255}),
              fillPattern=FillPattern.Solid)}), Documentation(info="<html>
<p>
The (time varying) Boolean output signal of this block can be defined in its
parameter menu via variable <strong>y</strong>. The purpose is to support the
easy definition of Boolean expressions in a block diagram. For example,
in the y-menu the definition \"time &gt;= 1 and time &lt;= 2\" can be given in order
to define that the output signal is <strong>true</strong> in the time interval
1 &le; time &le; 2 and otherwise it is <strong>false</strong>.
Note, that \"time\" is a built-in variable that is always
accessible and represents the \"model time\" and that
variable <strong>y</strong> is both a variable and a connector.
</p>
</html>"));

    end BooleanExpression;

    block ContinuousClock "Generate current time signal"
      extends Interfaces.SignalSource;

    equation
      y = offset + (if time < startTime then 0 else time - startTime);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Ellipse(extent={{-80,80},{80,-80}}, lineColor={160,160,164}),
            Line(points={{0,80},{0,60}}, color={160,160,164}),
            Line(points={{80,0},{60,0}}, color={160,160,164}),
            Line(points={{0,-80},{0,-60}}, color={160,160,164}),
            Line(points={{-80,0},{-60,0}}, color={160,160,164}),
            Line(points={{37,70},{26,50}}, color={160,160,164}),
            Line(points={{70,38},{49,26}}, color={160,160,164}),
            Line(points={{71,-37},{52,-27}}, color={160,160,164}),
            Line(points={{39,-70},{29,-51}}, color={160,160,164}),
            Line(points={{-39,-70},{-29,-52}}, color={160,160,164}),
            Line(points={{-71,-37},{-50,-26}}, color={160,160,164}),
            Line(points={{-71,37},{-54,28}}, color={160,160,164}),
            Line(points={{-38,70},{-28,51}}, color={160,160,164}),
            Line(
              points={{0,0},{-50,50}},
              thickness=0.5),
            Line(
              points={{0,0},{40,0}},
              thickness=0.5),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="startTime=%startTime")}),
        Documentation(info="<html>
<p>
The Real output y is a clock signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/ContinuousClock.png\"
     alt=\"ContinuousClock.png\">
</p>
</html>"));
    end ContinuousClock;

    block Constant "Generate constant signal of type Real"
      parameter Real k(start=1) "Constant output value"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Constant.png"));
      extends Interfaces.SO;

    equation
      y = k;
      annotation (
        defaultComponentName="const",
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,0},{80,0}}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="k=%k")}),
        Documentation(info="<html>
<p>
The Real output y is a constant signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Constant.png\"
     alt=\"Constant.png\">
</p>
</html>"));
    end Constant;

    block Step "Generate step signal of type Real"
      parameter Real height=1 "Height of step"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Step.png"));
      extends Interfaces.SignalSource;

    equation
      y = offset + (if time < startTime then 0 else height);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-70},{0,-70},{0,50},{80,50}}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="startTime=%startTime")}),
        Documentation(info="<html>
<p>
The Real output y is a step signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Step.png\"
     alt=\"Step.png\">
</p>

</html>"));
    end Step;

    block Ramp "Generate ramp signal"
      parameter Real height=1 "Height of ramps"
        annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Ramp.png"));
      parameter SI.Time duration(min=0.0, start=2)
        "Duration of ramp (= 0.0 gives a Step)";
      extends Interfaces.SignalSource;

    equation
      y = offset + (if time < startTime then 0 else if time < (startTime +
        duration) then (time - startTime)*height/duration else height);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-70},{-40,-70},{31,38}}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="duration=%duration"),
            Line(points={{31,38},{86,38}})}),
        Documentation(info="<html>
<p>
The Real output y is a ramp signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Ramp.png\"
     alt=\"Ramp.png\">
</p>

<p>
If parameter duration is set to 0.0, the limiting case of a Step signal is achieved.
</p>
</html>"));
    end Ramp;

    block Sine "Generate sine signal"
      import Modelica.Constants.pi;
      parameter Real amplitude=1 "Amplitude of sine wave"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Sine.png"));
      parameter SI.Frequency f(start=1) "Frequency of sine wave";
      parameter SI.Angle phase=0 "Phase of sine wave";
      extends Interfaces.SignalSource;
    equation
      y = offset + (if time < startTime then 0 else amplitude*Modelica.Math.sin(2
        *pi*f*(time - startTime) + phase));
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,0},{-68.7,34.2},{-61.5,53.1},{-55.1,66.4},{-49.4,
                  74.6},{-43.8,79.1},{-38.2,79.8},{-32.6,76.6},{-26.9,69.7},{-21.3,
                  59.4},{-14.9,44.1},{-6.83,21.2},{10.1,-30.8},{17.3,-50.2},{23.7,
                  -64.2},{29.3,-73.1},{35,-78.4},{40.6,-80},{46.2,-77.6},{51.9,-71.5},
                  {57.5,-61.9},{63.9,-47.2},{72,-24.8},{80,0}}, smooth = Smooth.Bezier),
            Text(
              extent={{-147,-152},{153,-112}},
              textString="f=%f")}),
        Documentation(info="<html>
<p>
The Real output y is a sine signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Sine.png\"
     alt=\"Sine.png\">
</p>
</html>"));
    end Sine;

    block Cosine "Generate cosine signal"
      import Modelica.Constants.pi;
      parameter Real amplitude=1 "Amplitude of cosine wave"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Cosine.png"));
      parameter SI.Frequency f(start=1) "Frequency of cosine wave";
      parameter SI.Angle phase=0 "Phase of cosine wave";
      extends Interfaces.SignalSource;
    equation
      y = offset + (if time < startTime then 0 else amplitude*Modelica.Math.cos(2
        *pi*f*(time - startTime) + phase));
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,80},{-76.2,79.8},{-70.6,76.6},{-64.9,69.7},{-59.3,
                  59.4},{-52.9,44.1},{-44.83,21.2},{-27.9,-30.8},{-20.7,-50.2},{-14.3,
                  -64.2},{-8.7,-73.1},{-3,-78.4},{2.6,-80},{8.2,-77.6},{13.9,-71.5},
                  {19.5,-61.9},{25.9,-47.2},{34,-24.8},{42,0}}, smooth=Smooth.Bezier),
            Text(
              extent={{-147,-152},{153,-112}},
              textString="f=%f"),
            Line(points={{42,1},{53.3,35.2},{60.5,54.1},{66.9,67.4},{72.6,75.6},{
                  78.2,80.1},{83.8,80.8}})}),
        Documentation(info="<html>
<p>
The Real output y is a cosine signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Cosine.png\"
     alt=\"Cosine.png\">
</p>
</html>"));
    end Cosine;

    block SineVariableFrequencyAndAmplitude
      "Generate sine signal with variable frequency and amplitude"
      extends Interfaces.SO;
      import Modelica.Constants.pi;
      parameter Boolean useConstantAmplitude=false "Enable constant amplitude";
      parameter Real constantAmplitude=1 "Constant amplitude"
        annotation(Dialog(enable=useConstantAmplitude));
      parameter Boolean useConstantFrequency=false "Enable constant frequency";
      parameter SI.Frequency constantFrequency=1 "Constant frequency"
        annotation(Dialog(enable=useConstantFrequency));
      parameter Real offset=0 "Offset of the sine wave"
        annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/SineVariableFrequencyAndAmplitude.png"));
      SI.Angle phi(start=0) "Phase of the sine wave";
      Blocks.Interfaces.RealInput amplitude if not useConstantAmplitude
        "Amplitude" annotation (Placement(transformation(extent={{-20,-20},{20,
                20}}, origin={-120,60})));
      Blocks.Interfaces.RealInput f(unit="Hz") if not useConstantFrequency
        "Frequency" annotation (Placement(transformation(extent={{-20,-20},{20,
                20}}, origin={-120,-60})));
    protected
      Blocks.Interfaces.RealInput amplitude_internal "Amplitude" annotation (
          Placement(transformation(extent={{-2,-2},{2,2}}, origin={-80,60})));
      Blocks.Interfaces.RealInput f_internal(unit="Hz") "Frequency" annotation
        (Placement(transformation(extent={{-2,-2},{2,2}}, origin={-80,-60})));
      Blocks.Sources.Constant amplitude_constant(final k=constantAmplitude)
        if useConstantAmplitude annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-80,30})));
      Blocks.Sources.Constant f_constant(final k=constantFrequency)
        if useConstantFrequency annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-80,-30})));
    equation
      der(phi) = 2*pi*f_internal;
      y = offset + amplitude_internal*sin(phi);
      connect(f, f_internal)
        annotation (Line(points={{-120,-60},{-80,-60}}, color={0,0,127}));
      connect(amplitude, amplitude_internal)
        annotation (Line(points={{-120,60},{-80,60}}, color={0,0,127}));
      connect(amplitude_constant.y, amplitude_internal)
        annotation (Line(points={{-80,41},{-80,60}}, color={0,0,127}));
      connect(f_constant.y, f_internal)
        annotation (Line(points={{-80,-41},{-80,-60}}, color={0,0,127}));
      annotation (defaultComponentName="sine",
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={
    {-80,0},{-78.4,0},{-76.8,0},{-75.2,0},{-73.6,0.1},
    {-72,0.1},{-70.4,0.2},{-68.8,0.3},{-67.2,0.4},{-65.6,0.6},
    {-64,0.8},{-62.4,1.1},{-60.8,1.4},{-59.2,1.8},{-57.6,2.2},
    {-56,2.7},{-54.4,3.3},{-52.8,3.9},{-51.2,4.6},{-49.6,5.4},
    {-48,6.2},{-46.4,7.2},{-44.8,8.2},{-43.2,9.2},{-41.6,10.4},
    {-40,11.6},{-38.4,12.9},{-36.8,14.2},{-35.2,15.6},{-33.6,17.1},
    {-32,18.6},{-30.4,20.1},{-28.8,21.6},{-27.2,23.1},{-25.6,24.6},
    {-24,26.1},{-22.4,27.5},{-20.8,28.8},{-19.2,30},{-17.6,31.1},
    {-16,32},{-14.4,32.7},{-12.8,33.2},{-11.2,33.5},{-9.6,33.5},
    {-8,33.2},{-6.4,32.5},{-4.8,31.5},{-3.2,30.1},{-1.6,28.4},
    {0,26.2},{1.6,23.6},{3.2,20.6},{4.8,17.2},{6.4,13.3},
    {8,9.1},{9.6,4.6},{11.2,-0.3},{12.8,-5.4},{14.4,-10.7},
    {16,-16.1},{17.6,-21.6},{19.2,-27.1},{20.8,-32.3},{22.4,-37.4},
    {24,-42.1},{25.6,-46.3},{27.2,-49.9},{28.8,-52.8},{30.4,-54.8},
    {32,-56},{33.6,-56.1},{35.2,-55.2},{36.8,-53.1},{38.4,-49.8},
    {40,-45.3},{41.6,-39.7},{43.2,-33},{44.8,-25.3},{46.4,-16.6},
    {48,-7.3},{49.6,2.6},{51.2,12.8},{52.8,23},{54.4,33},
    {56,42.5},{57.6,51.2},{59.2,58.8},{60.8,64.9},{62.4,69.3},
    {64,71.9},{65.6,72.3},{67.2,70.5},{68.8,66.4},{70.4,60},
    {72,51.4},{73.6,40.8},{75.2,28.4},{76.8,14.7},{78.4,0},
    {80,-15.1}},     smooth = Smooth.Bezier)}),
        Documentation(info="<html>
<p>
This signal source provides a sinusoidal signal with variable frequency <code>f</code> and variable <code>amplitude</code>,
i.e. the phase angle of the sine wave is integrated from 2*&pi;*f.
</p>
<p>
Note that the initial value of the phase angle <code>phi</code> defines the initial phase shift,
and that the parameter <code>startTime</code> is omitted since the voltage can be kept equal to offset with setting the input <code>amplitude</code> to zero.
</p>
</html>"));
    end SineVariableFrequencyAndAmplitude;

    block CosineVariableFrequencyAndAmplitude
      "Generate cosine signal with variable frequency and amplitude"
      extends Interfaces.SO;
      import Modelica.Constants.pi;
      parameter Boolean useConstantAmplitude=false "Enable constant amplitude";
      parameter Real constantAmplitude=1 "Constant amplitude"
        annotation(Dialog(enable=useConstantAmplitude));
      parameter Boolean useConstantFrequency=false "Enable constant frequency";
      parameter SI.Frequency constantFrequency=1 "Constant frequency"
        annotation(Dialog(enable=useConstantFrequency));
      parameter Real offset=0 "Offset of the sine wave"
        annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/CosineVariableFrequencyAndAmplitude.png"));
      SI.Angle phi(start=0) "Phase of the sine wave";
      Blocks.Interfaces.RealInput amplitude if not useConstantAmplitude
        "Amplitude" annotation (Placement(transformation(extent={{-20,-20},{20,
                20}}, origin={-120,60})));
      Blocks.Interfaces.RealInput f(unit="Hz") if not useConstantFrequency
        "Frequency" annotation (Placement(transformation(extent={{-20,-20},{20,
                20}}, origin={-120,-60})));
    protected
      Blocks.Interfaces.RealInput amplitude_internal "Amplitude" annotation (
          Placement(transformation(extent={{-2,-2},{2,2}}, origin={-80,60})));
      Blocks.Interfaces.RealInput f_internal(unit="Hz") "Frequency" annotation
        (Placement(transformation(extent={{-2,-2},{2,2}}, origin={-80,-60})));
      Blocks.Sources.Constant amplitude_constant(final k=constantAmplitude)
        if useConstantAmplitude annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-80,30})));
      Blocks.Sources.Constant f_constant(final k=constantFrequency)
        if useConstantFrequency annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={-80,-30})));
    equation
      der(phi) = 2*pi*f_internal;
      y = offset + amplitude_internal*cos(phi);
      connect(f, f_internal)
        annotation (Line(points={{-120,-60},{-80,-60}}, color={0,0,127}));
      connect(amplitude, amplitude_internal)
        annotation (Line(points={{-120,60},{-80,60}}, color={0,0,127}));
      connect(amplitude_constant.y, amplitude_internal)
        annotation (Line(points={{-80,41},{-80,60}}, color={0,0,127}));
      connect(f_constant.y, f_internal)
        annotation (Line(points={{-80,-41},{-80,-60}}, color={0,0,127}));
      annotation (defaultComponentName="cosine",
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={
    {-80,80},{-78.4,79.6},{-76.8,79.2},{-75.2,78.8},{-73.6,78.4},{-72,78},
    {-70.4,77.5},{-68.8,77.1},{-67.2,76.6},{-65.6,76.1},{-64,75.6},
    {-62.4,75},{-60.8,74.4},{-59.2,73.7},{-57.6,73},{-56,72.2},
    {-54.4,71.3},{-52.8,70.3},{-51.2,69.2},{-49.6,68},{-48,66.6},
    {-46.4,65.2},{-44.8,63.6},{-43.2,61.8},{-41.6,59.9},{-40,57.7},
    {-38.4,55.5},{-36.8,53},{-35.2,50.3},{-33.6,47.5},{-32,44.4},
    {-30.4,41.1},{-28.8,37.7},{-27.2,34},{-25.6,30.1},{-24,26.1},
    {-22.4,21.9},{-20.8,17.5},{-19.2,13},{-17.6,8.3},{-16,3.5},
    {-14.4,-1.3},{-12.8,-6.2},{-11.2,-11.1},{-9.6,-16},{-8,-20.8},
    {-6.4,-25.5},{-4.8,-30.1},{-3.2,-34.5},{-1.6,-38.6},{0,-42.4},
    {1.6,-45.9},{3.2,-49},{4.8,-51.7},{6.4,-53.9},{8,-55.5},
    {9.6,-56.5},{11.2,-57},{12.8,-56.8},{14.4,-55.9},{16,-54.4},
    {17.6,-52.2},{19.2,-49.3},{20.8,-45.7},{22.4,-41.5},{24,-36.7},
    {25.6,-31.4},{27.2,-25.6},{28.8,-19.4},{30.4,-12.9},{32,-6.2},
    {33.6,0.6},{35.2,7.4},{36.8,14},{38.4,20.4},{40,26.3},
    {41.6,31.8},{43.2,36.5},{44.8,40.6},{46.4,43.7},{48,45.9},
    {49.6,47.1},{51.2,47.2},{52.8,46.2},{54.4,44.1},{56,41},
    {57.6,36.8},{59.2,31.8},{60.8,25.9},{62.4,19.4},{64,12.4},
    {65.6,5.1},{67.2,-2.2},{68.8,-9.5},{70.4,-16.4},{72,-22.8},
    {73.6,-28.4},{75.2,-33},{76.8,-36.6},{78.4,-38.9},{80,-39.8}},
        smooth = Smooth.Bezier)}),
        Documentation(info="<html>
<p>
This signal source provides a cosine signal with variable frequency <code>f</code> and variable <code>amplitude</code>,
i.e. the phase angle of the cosine wave is integrated from 2*&pi;*f.
</p>
<p>
Note that the initial value of the phase angle <code>phi</code> defines the initial phase shift,
and that the parameter <code>startTime</code> is omitted since the voltage can be kept equal to offset with setting the input <code>amplitude</code> to zero.
</p>
</html>"));
    end CosineVariableFrequencyAndAmplitude;

    block Sinc "Generate sinc signal"
      import Modelica.Constants.pi;
      import Modelica.Constants.eps;
      parameter Real amplitude=1 "Amplitude of sine wave"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Sinc.png"));
      parameter SI.Frequency f(start=1) "Frequency of sine wave";
      extends Interfaces.SignalSource;
    protected
      SI.Angle x=2*pi*f*(time - startTime);
    equation
      y = offset + (if time < startTime then 0 else amplitude*
        (if noEvent(time - startTime < eps) then 1 else (sin(x))/x));
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-147,-152},{153,-112}},
              textString="f=%f",
              textColor={0,0,0}),
        Line(points={
          {-80, 80.0},{-76, 78.7},{-72, 74.8},{-68, 68.7},{-64, 60.5},
          {-60, 50.9},{-56, 40.4},{-52, 29.4},{-48, 18.7},{-44,  8.7},
          {-40,  0.0},{-36, -7.2},{-32,-12.5},{-28,-15.8},{-24,-17.3},
          {-20,-17.0},{-16,-15.1},{-12,-12.1},{ -8, -8.3},{ -4, -4.1},
          {  0,  0.0},{  4,  3.7},{  8,  6.8},{ 12,  9.0},{ 16, 10.1},
          { 20, 10.2},{ 24,  9.3},{ 28,  7.6},{ 32,  5.3},{ 36,  2.7},
          { 40,  0.0},{ 44, -2.5},{ 48, -4.7},{ 52, -6.2},{ 56, -7.1},
          { 60, -7.3},{ 64, -6.7},{ 68, -5.6},{ 72, -3.9},{ 76, -2.0},
          { 80,  0.0}}, smooth = Smooth.Bezier)}),
        Documentation(info="<html>
<p>
The Real output y is a sinc signal: <code> amplitude*(sin(2*&pi;*f*t))/((2*&pi;*f*t))</code>
</p>
<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Sinc.png\"
     alt=\"Sinc.png\">
</p>
</html>"));
    end Sinc;

    block ExpSine "Generate exponentially damped sine signal"
      import Modelica.Constants.pi;
      parameter Real amplitude=1 "Amplitude of sine wave"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/ExpSine.png"));
      parameter SI.Frequency f(start=2) "Frequency of sine wave";
      parameter SI.Angle phase=0 "Phase of sine wave";
      parameter SI.Damping damping(start=1)
        "Damping coefficient of sine wave";
      extends Interfaces.SignalSource;
    equation
      y = offset + (if time < startTime then 0 else amplitude*Modelica.Math.exp(-
        (time - startTime)*damping)*Modelica.Math.sin(2*pi*f*(time -
        startTime) + phase));
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,0},{-75.2,32.3},{-72,50.3},{-68.7,64.5},{-65.5,74.2},
                  {-62.3,79.3},{-59.1,79.6},{-55.9,75.3},{-52.7,67.1},{-48.6,52.2},
                  {-43,25.8},{-35,-13.9},{-30.2,-33.7},{-26.1,-45.9},{-22.1,-53.2},
                  {-18.1,-55.3},{-14.1,-52.5},{-10.1,-45.3},{-5.23,-32.1},{8.44,
                  13.7},{13.3,26.4},{18.1,34.8},{22.1,38},{26.9,37.2},{31.8,31.8},
                  {38.2,19.4},{51.1,-10.5},{57.5,-21.2},{63.1,-25.9},{68.7,-25.9},
                  {75.2,-20.5},{80,-13.8}}, smooth = Smooth.Bezier),
            Text(
              extent={{-147,-152},{153,-112}},
              textString="f=%f")}),
        Documentation(info="<html>
<p>
The Real output y is a sine signal with exponentially changing amplitude:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/ExpSine.png\"
     alt=\"ExpSine.png\">
</p>
</html>"));
    end ExpSine;

    block Exponentials "Generate a rising and falling exponential signal"

      parameter Real outMax=1 "Height of output for infinite riseTime"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Exponentials.png"));
      parameter SI.Time riseTime(min=0,start=0.5) "Rise time";
      parameter SI.Time riseTimeConst(min=Modelica.Constants.small) = 0.1
        "Rise time constant; rising is defined as outMax*(1-exp(-riseTime/riseTimeConst))";
      parameter SI.Time fallTimeConst(min=Modelica.Constants.small)=
        riseTimeConst "Fall time constant";
      extends Interfaces.SignalSource;
    protected
      Real y_riseTime;

    equation
      y_riseTime = outMax*(1 - Modelica.Math.exp(-riseTime/riseTimeConst));
      y = offset + (if (time < startTime) then 0 else if (time < (startTime +
        riseTime)) then outMax*(1 - Modelica.Math.exp(-(time - startTime)/
        riseTimeConst)) else y_riseTime*Modelica.Math.exp(-(time - startTime -
        riseTime)/fallTimeConst));

      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-90,-70},{68,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-70},{-77.2,-55.3},{-74.3,-42.1},{-70.8,-27.6},{-67.3,
                  -15},{-63.7,-4.08},{-59.5,7.18},{-55.3,16.7},{-50.3,26},{-44.6,
                  34.5},{-38.3,42.1},{-31.2,48.6},{-22.7,54.3},{-12.1,59.2},{-10,
                  60},{-7.88,47.5},{-5.05,32.7},{-2.22,19.8},{0.606,8.45},{4.14,-3.7},
                  {7.68,-14},{11.9,-24.2},{16.2,-32.6},{21.1,-40.5},{26.8,-47.4},
                  {33.1,-53.3},{40.9,-58.5},{50.8,-62.8},{60,-65.4}}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="riseTime=%riseTime")}),
        Documentation(info="<html>
<p>
The Real output y is a rising exponential followed
by a falling exponential signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Exponentials.png\"
     alt=\"Exponentials.png\">
</p>
</html>"));
    end Exponentials;

    block Pulse "Generate pulse signal of type Real"
      parameter Real amplitude=1 "Amplitude of pulse"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Pulse.png"));
      parameter Real width(
        final min=Modelica.Constants.small,
        final max=100) = 50 "Width of pulse in % of period";
      parameter SI.Time period(final min=Modelica.Constants.small,
          start=1) "Time for one period";
      parameter Integer nperiod=-1
        "Number of periods (< 0 means infinite number of periods)";
      extends Interfaces.SignalSource;
    protected
      SI.Time T_width=period*width/100;
      SI.Time T_start "Start time of current period";
      Integer count "Period count";
    initial algorithm
      count := integer((time - startTime)/period);
      T_start := startTime + count*period;
    equation
      when integer((time - startTime)/period) > pre(count) then
        count = pre(count) + 1;
        T_start = time;
      end when;
      y = offset + (if (time < startTime or nperiod == 0 or (nperiod > 0 and
        count >= nperiod)) then 0 else if time < T_start + T_width then amplitude
         else 0);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-70},{-40,-70},{-40,44},{0,44},{0,-70},{40,-70},{40,
                  44},{79,44}}),
            Text(
              extent={{-147,-152},{153,-112}},
              textString="period=%period")}),
        Documentation(info="<html>
<p>
The Real output y is a pulse signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Pulse.png\"
     alt=\"Pulse.png\">
</p>
</html>"));
    end Pulse;

    block SawTooth "Generate saw tooth signal"
      parameter Real amplitude=1 "Amplitude of saw tooth"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/SawTooth.png"));
      parameter SI.Time period(final min=Modelica.Constants.small,start=1)
        "Time for one period";
      parameter Integer nperiod=-1
        "Number of periods (< 0 means infinite number of periods)";
      extends Interfaces.SignalSource;
    protected
      SI.Time T_start(final start=startTime) "Start time of current period";
      Integer count "Period count";
    initial algorithm
      count := integer((time - startTime)/period);
      T_start := startTime + count*period;
    equation
      when integer((time - startTime)/period) > pre(count) then
        count = pre(count) + 1;
        T_start = time;
      end when;
      y = offset + (if (time < startTime or nperiod == 0 or (nperiod > 0 and
        count >= nperiod)) then 0 else amplitude*(time - T_start)/period);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-70},{-60,-70},{0,40},{0,-70},{60,41},{60,-70}}),
            Text(
              extent={{-147,-152},{153,-112}},
              textString="period=%period")}),
        Documentation(info="<html>
<p>
The Real output y is a saw tooth signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/SawTooth.png\"
     alt=\"SawTooth.png\">
</p>
</html>"));
    end SawTooth;

    block Trapezoid "Generate trapezoidal signal of type Real"
      parameter Real amplitude=1 "Amplitude of trapezoid"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/Trapezoid.png"));
      parameter SI.Time rising(final min=0) = 0
        "Rising duration of trapezoid";
      parameter SI.Time width(final min=0) = 0.5
        "Width duration of trapezoid";
      parameter SI.Time falling(final min=0) = 0
        "Falling duration of trapezoid";
      parameter SI.Time period(final min=Modelica.Constants.small, start=1)
        "Time for one period";
      parameter Integer nperiod=-1
        "Number of periods (< 0 means infinite number of periods)";
      extends Interfaces.SignalSource;
    protected
      parameter SI.Time T_rising=rising
        "End time of rising phase within one period";
      parameter SI.Time T_width=T_rising + width
        "End time of width phase within one period";
      parameter SI.Time T_falling=T_width + falling
        "End time of falling phase within one period";
      SI.Time T_start "Start time of current period";
      Integer count "Period count";
    initial algorithm
      count := integer((time - startTime)/period);
      T_start := startTime + count*period;
    equation
      when integer((time - startTime)/period) > pre(count) then
        count = pre(count) + 1;
        T_start = time;
      end when;
      y = offset + (if (time < startTime or nperiod == 0 or (nperiod > 0 and
        count >= nperiod)) then 0 else if (time < T_start + T_rising) then
        amplitude*(time - T_start)/rising else if (time < T_start + T_width)
         then amplitude else if (time < T_start + T_falling) then amplitude*(
        T_start + T_falling - time)/falling else 0);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-147,-152},{153,-112}},
              textString="period=%period"),
            Line(points={{-81,-70},{-60,-70},{-30,40},{9,40},{39,-70},{61,-70},{
                  90,40}})}),
        Documentation(info="<html>
<p>
The Real output y is a trapezoid signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Trapezoid.png\"
     alt=\"Trapezoid\">
</p>
</html>"));
    end Trapezoid;

    block LogFrequencySweep "Logarithmic frequency sweep"
      extends Blocks.Interfaces.SO;
      import Modelica.Constants.eps;
      parameter Real wMin(final min=eps) "Start frequency"
        annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/LogFrequencySweep.png"));
      parameter Real wMax(final min=eps) "End frequency";
      parameter SI.Time startTime=0 "Start time of frequency sweep";
      parameter SI.Time duration(min=0.0, start=1) "Duration of ramp (= 0.0 gives a Step)";
    equation
      y = if time < startTime then wMin else
        if time < (startTime + max(duration,eps)) then
          10^(log10(wMin) + (log10(wMax) - log10(wMin))*min(1, (time-startTime)/max(duration,eps)))
        else
          wMax;
       annotation (defaultComponentName="logSweep",
         Documentation(info="<html>
<p>The output <code>y</code> performs a logarithmic frequency sweep.
The logarithm of frequency <code>w</code> performs a linear ramp from <code>log10(wMin)</code> to <code>log10(wMax)</code>.
The output is the decimal power of this logarithmic ramp.
</p>
<p>For <code>time &lt; startTime</code> the output is equal to <code>wMin</code>.</p>
<p>For <code>time &gt; startTime+duration</code> the output is equal to <code>wMax</code>.</p>
<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/LogFrequencySweep.png\"
     alt=\"LogFrequencySweep.png\">
</p>

</html>"),
        Icon(graphics={
            Line(points={{-78,44},{80,44}}, color={192,192,192}),
            Line(points={{-78,34},{80,34}}, color={192,192,192}),
            Line(points={{-78,20},{80,20}}, color={192,192,192}),
            Line(points={{-78,-2},{80,-2}}, color={192,192,192}),
            Line(points={{-78,-48},{80,-48}}, color={192,192,192}),
            Line(
              points={{-70,-48},{-50,-48},{50,44},{70,44}},
              color={0,0,127},
              thickness=0.5),
            Line(points={{-50,-48},{-50,44}}, color={192,192,192}),
            Line(points={{50,-48},{50,44}}, color={192,192,192}),
            Line(points={{-78,40},{80,40}}, color={192,192,192}),
                                   Polygon(
                  points={{90,-48},{68,-40},{68,-56},{90,-48}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                            Polygon(
                  points={{-70,90},{-78,68},{-62,68},{-70,90}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
            Line(points={{-70,-56},{-70,68}}, color={192,192,192})}));
    end LogFrequencySweep;

    block KinematicPTP
      "Move as fast as possible along a distance within given kinematic constraints"

      parameter Real deltaq[:]={1} "Distance to move"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/KinematicPTP.png"));
      parameter Real qd_max[:](each final min=Modelica.Constants.small) = {1}
        "Maximum velocities der(q)";
      parameter Real qdd_max[:](each final min=Modelica.Constants.small) = {1}
        "Maximum accelerations der(qd)";
      parameter SI.Time startTime=0 "Time instant at which movement starts";

      extends Interfaces.MO(final nout=max([size(deltaq, 1); size(qd_max, 1); size(qdd_max, 1)]));

    protected
      parameter Real p_deltaq[nout]=(if size(deltaq, 1) == 1 then ones(nout)*
          deltaq[1] else deltaq);
      parameter Real p_qd_max[nout]=(if size(qd_max, 1) == 1 then ones(nout)*
          qd_max[1] else qd_max);
      parameter Real p_qdd_max[nout]=(if size(qdd_max, 1) == 1 then ones(nout)*
          qdd_max[1] else qdd_max);
      Real sd_max;
      Real sdd_max;
      Real sdd;
      Real aux1[nout];
      Real aux2[nout];
      SI.Time Ta1;
      SI.Time Ta2;
      SI.Time Tv;
      SI.Time Te;
      Boolean noWphase;

    equation
      for i in 1:nout loop
        aux1[i] = p_deltaq[i]/p_qd_max[i];
        aux2[i] = p_deltaq[i]/p_qdd_max[i];
      end for;
      sd_max = 1/max(abs(aux1));
      sdd_max = 1/max(abs(aux2));

      Ta1 = sqrt(1/sdd_max);
      Ta2 = sd_max/sdd_max;
      noWphase = Ta2 >= Ta1;
      Tv = if noWphase then Ta1 else 1/sd_max;
      Te = if noWphase then Ta1 + Ta1 else Tv + Ta2;

      // path-acceleration
      sdd = if time < startTime then 0 else ((if noWphase then (if time < Ta1 +
        startTime then sdd_max else (if time < Te + startTime then -sdd_max else
        0)) else (if time < Ta2 + startTime then sdd_max else (if time < Tv +
        startTime then 0 else (if time < Te + startTime then -sdd_max else 0)))));

      // acceleration
      y = p_deltaq*sdd;
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,78},{-80,-82}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,88},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{82,0}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-80,0},{-70,0},{-70,70},{-30,70},{-30,0},{20,0},{20,-70},{
                  60,-70},{60,0},{68,0}}),
            Text(
              extent={{2,80},{80,20}},
              textColor={192,192,192},
              textString="acc"),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="deltaq=%deltaq")}),
            Documentation(info="<html>
<p>
The goal is to move as <strong>fast</strong> as possible along a distance
<strong>deltaq</strong>
under given <strong>kinematical constraints</strong>. The distance can be a positional or
angular range. In robotics such a movement is called <strong>PTP</strong> (Point-To-Point).
This source block generates the <strong>acceleration</strong> qdd of this signal
as output:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/KinematicPTP.png\"
     alt=\"KinematicPTP.png\">
</p>

<p>
After integrating the output two times, the position q is
obtained. The signal is constructed in such a way that it is not possible
to move faster, given the <strong>maximally</strong> allowed <strong>velocity</strong> qd_max and
the <strong>maximally</strong> allowed <strong>acceleration</strong> qdd_max.
</p>
<p>
If several distances are given (vector deltaq has more than 1 element),
an acceleration output vector is constructed such that all signals
are in the same periods in the acceleration, constant velocity
and deceleration phase. This means that only one of the signals
is at its limits whereas the others are synchronized in such a way
that the end point is reached at the same time instant.
</p>

<p>
This element is useful to generate a reference signal for a controller
which controls a drive train or in combination with model
Modelica.Mechanics.Rotational.<strong>Accelerate</strong> to drive
a flange according to a given acceleration.
</p>

</html>",   revisions="<html>
<p><strong>Release Notes:</strong></p>
<ul>
<li><em>June 27, 2001</em>
       by Bernhard Bachmann.<br>
       Bug fixed that element is also correct if startTime is not zero.</li>
<li><em>Nov. 3, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Vectorized and moved from Rotational to Blocks.Sources.</li>
<li><em>June 29, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       realized.</li>
</ul>
</html>"));
    end KinematicPTP;

    block KinematicPTP2
      "Move as fast as possible from start to end position within given kinematic constraints with output signals q, qd=der(q), qdd=der(qd)"

      parameter Real q_begin[:]={0} "Start position"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/KinematicPTP2.png"));
      parameter Real q_end[:]={1} "End position";
      parameter Real qd_max[:](each final min=Modelica.Constants.small) = {1}
        "Maximum velocities der(q)";
      parameter Real qdd_max[:](each final min=Modelica.Constants.small) = {1}
        "Maximum accelerations der(qd)";
      parameter SI.Time startTime=0
        "Time instant at which movement starts";

      extends Blocks.Icons.Block;
      final parameter Integer nout=max([size(q_begin, 1); size(q_end, 1); size(qd_max, 1); size(qdd_max, 1)])
        "Number of output signals (= dimension of q, qd, qdd, moving)";
      output SI.Time endTime "Time instant at which movement stops";

      Blocks.Interfaces.RealOutput q[nout]
        "Reference position of path planning"
        annotation (Placement(transformation(extent={{100,70},{120,90}})));
      Blocks.Interfaces.RealOutput qd[nout] "Reference speed of path planning"
        annotation (Placement(transformation(extent={{100,20},{120,40}})));
      Blocks.Interfaces.RealOutput qdd[nout]
        "Reference acceleration of path planning"
        annotation (Placement(transformation(extent={{100,-40},{120,-20}})));
      Blocks.Interfaces.BooleanOutput moving[nout]
        "= true, if end position not yet reached; = false, if end position reached or axis is completely at rest"
        annotation (Placement(transformation(extent={{100,-90},{120,-70}})));

    protected
      parameter Real p_q_begin[nout]=(if size(q_begin, 1) == 1 then ones(nout)*
          q_begin[1] else q_begin);
      parameter Real p_q_end[nout]=(if size(q_end, 1) == 1 then ones(nout)*q_end[
          1] else q_end);
      parameter Real p_qd_max[nout]=(if size(qd_max, 1) == 1 then ones(nout)*
          qd_max[1] else qd_max);
      parameter Real p_qdd_max[nout]=(if size(qdd_max, 1) == 1 then ones(nout)*
          qdd_max[1] else qdd_max);
      parameter Real p_deltaq[nout]=p_q_end - p_q_begin;
      constant Real eps=10*Modelica.Constants.eps;
      Boolean motion_ref;
      Real sd_max_inv;
      Real sdd_max_inv;
      Real sd_max;
      Real sdd_max;
      Real sdd;
      Real aux1[nout];
      Real aux2[nout];
      SI.Time Ta1;
      SI.Time Ta2;
      SI.Time Tv;
      SI.Time Te;
      Boolean noWphase;
      SI.Time Ta1s;
      SI.Time Ta2s;
      SI.Time Tvs;
      SI.Time Tes;
      Real sd_max2;
      Real s1;
      Real s2;
      Real s3;
      Real s;
      Real sd;

    equation
      for i in 1:nout loop
        aux1[i] = p_deltaq[i]/p_qd_max[i];
        aux2[i] = p_deltaq[i]/p_qdd_max[i];
      end for;

      sd_max_inv = max(abs(aux1));
      sdd_max_inv = max(abs(aux2));

      if sd_max_inv <= eps or sdd_max_inv <= eps then
        sd_max = 0;
        sdd_max = 0;
        Ta1 = 0;
        Ta2 = 0;
        noWphase = false;
        Tv = 0;
        Te = 0;
        Ta1s = 0;
        Ta2s = 0;
        Tvs = 0;
        Tes = 0;
        sd_max2 = 0;
        s1 = 0;
        s2 = 0;
        s3 = 0;
        s = 0;
      else
        sd_max = 1/max(abs(aux1));
        sdd_max = 1/max(abs(aux2));
        Ta1 = sqrt(1/sdd_max);
        Ta2 = sd_max/sdd_max;
        noWphase = Ta2 >= Ta1;
        Tv = if noWphase then Ta1 else 1/sd_max;
        Te = if noWphase then Ta1 + Ta1 else Tv + Ta2;
        Ta1s = Ta1 + startTime;
        Ta2s = Ta2 + startTime;
        Tvs = Tv + startTime;
        Tes = Te + startTime;
        sd_max2 = sdd_max*Ta1;
        s1 = sdd_max*(if noWphase then Ta1*Ta1 else Ta2*Ta2)/2;
        s2 = s1 + (if noWphase then sd_max2*(Te - Ta1) - (sdd_max/2)*(Te - Ta1)^2
           else sd_max*(Tv - Ta2));
        s3 = s2 + sd_max*(Te - Tv) - (sdd_max/2)*(Te - Tv)*(Te - Tv);

        if time < startTime then
          s = 0;
        elseif noWphase then
          if time < Ta1s then
            s = (sdd_max/2)*(time - startTime)*(time - startTime);
          elseif time < Tes then
            s = s1 + sd_max2*(time - Ta1s) - (sdd_max/2)*(time - Ta1s)*(time -
              Ta1s);
          else
            s = s2;
          end if;
        elseif time < Ta2s then
          s = (sdd_max/2)*(time - startTime)*(time - startTime);
        elseif time < Tvs then
          s = s1 + sd_max*(time - Ta2s);
        elseif time < Tes then
          s = s2 + sd_max*(time - Tvs) - (sdd_max/2)*(time - Tvs)*(time - Tvs);
        else
          s = s3;
        end if;

      end if;

      sd = der(s);
      sdd = der(sd);

      qdd = p_deltaq*sdd;
      qd = p_deltaq*sd;
      q = p_q_begin + p_deltaq*s;
      endTime = Tes;

      // report when axis is moving
      motion_ref = time < endTime;
      for i in 1:nout loop
        moving[i] = if abs(q_begin[i] - q_end[i]) > eps then motion_ref else
          false;
      end for;

      annotation (
        defaultComponentName="kinematicPTP",
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,78},{-80,-82}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,88},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{17,0}}, color={192,192,192}),
            Line(
              points={{-80,0},{-70,0},{-70,70},{-50,70},{-50,0},{-15,0},{-15,-70},
                  {5,-70},{5,0},{18,0}}),
            Text(
              extent={{34,96},{94,66}},
              textString="q"),
            Text(
              extent={{40,44},{96,14}},
              textString="qd"),
            Text(
              extent={{32,-18},{99,-44}},
              textString="qdd"),
            Text(
              extent={{-32,-74},{97,-96}},
              textString="moving")}),
        Documentation(info="<html>
<p>
The goal is to move as <strong>fast</strong> as possible from start position <strong>q_begin</strong>
to end position <strong>q_end</strong>
under given <strong>kinematical constraints</strong>. The positions can be translational or
rotational definitions (i.e., q_begin/q_end is given). In robotics such a movement is called <strong>PTP</strong> (Point-To-Point).
This source block generates the <strong>position</strong> q(t), the
<strong>speed</strong> qd(t) = der(q), and the <strong>acceleration</strong> qdd = der(qd)
as output. The signals are constructed in such a way that it is not possible
to move faster, given the <strong>maximally</strong> allowed <strong>velocity</strong> qd_max and
the <strong>maximally</strong> allowed <strong>acceleration</strong> qdd_max:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/KinematicPTP2.png\"
     alt=\"KinematicPTP2.png\">
</p>

<p>
If vectors q_begin/q_end have more than 1 element,
the output vectors are constructed such that all signals
are in the same periods in the acceleration, constant velocity
and deceleration phase. This means that only one of the signals
is at its limits whereas the others are synchronized in such a way
that the end point is reached at the same time instant.
</p>

<p>
This element is useful to generate a reference signal for a controller
which controls, e.g., a drive train, or to drive
a flange according to a given acceleration.
</p>

</html>",   revisions="<html>
<ul>
<li><em>March 24, 2007</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Non-standard Modelica function \"constrain(..)\" replaced by standard
       Modelica implementation (via internal function position()).<br>
       New output signal \"moving\" added.</li>
<li><em>June 27, 2001</em>
       by Bernhard Bachmann.<br>
       Bug fixed that element is also correct if startTime is not zero.</li>
<li><em>Nov. 3, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Vectorized and moved from Rotational to Blocks.Sources.</li>
<li><em>June 29, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       realized.</li>
</ul>
</html>"));
    end KinematicPTP2;

    block TimeTable
      "Generate a (possibly discontinuous) signal by linear interpolation in a table"

      parameter Real table[:, 2] = fill(0.0, 0, 2)
        "Table matrix (time = first column; e.g., table=[0, 0; 1, 1; 2, 4])"
        annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/TimeTable.png"));
      parameter SI.Time timeScale(
        min=Modelica.Constants.eps)=1 "Time scale of first table column"
        annotation (Evaluate=true);
      extends Interfaces.SignalSource;
      parameter SI.Time shiftTime=startTime
        "Shift time of first table column";
    protected
      discrete Real a "Interpolation coefficient a of actual interval (y=a*x+b)";
      discrete Real b "Interpolation coefficient b of actual interval (y=a*x+b)";
      Integer last(start=1) "Last used lower grid index";
      discrete SI.Time nextEvent(start=0, fixed=true) "Next event instant";
      discrete Real nextEventScaled(start=0, fixed=true)
        "Next scaled event instant";
      Real timeScaled "Scaled time";

      function getInterpolationCoefficients
        "Determine interpolation coefficients and next time event"
        extends Modelica.Icons.Function;
        input Real table[:, 2] "Table for interpolation";
        input Real offset "y-offset";
        input Real startTimeScaled "Scaled time-offset";
        input Real timeScaled "Actual scaled time instant";
        input Integer last "Last used lower grid index";
        input Real TimeEps "Relative epsilon to check for identical time instants";
        input Real shiftTimeScaled "Time shift";
        output Real a "Interpolation coefficient a (y=a*x + b)";
        output Real b "Interpolation coefficient b (y=a*x + b)";
        output Real nextEventScaled "Next scaled event instant";
        output Integer next "New lower grid index";
      protected
        Integer columns=2 "Column to be interpolated";
        Integer ncol=2 "Number of columns to be interpolated";
        Integer nrow=size(table, 1) "Number of table rows";
        Integer next0;
        Real tp;
        Real dt;
      algorithm
        next := last;
        nextEventScaled := timeScaled - TimeEps*abs(timeScaled);
        // in case there are no more time events
        tp := timeScaled + TimeEps*abs(timeScaled);

        if tp < startTimeScaled then
          // First event not yet reached
          nextEventScaled := startTimeScaled;
          a := 0;
          b := offset;
        elseif nrow < 2 then
          // Special action if table has only one row
          a := 0;
          b := offset + table[1, columns];
        else
          tp := tp - shiftTimeScaled;
          // Find next time event instant. Note, that two consecutive time instants
          // in the table may be identical due to a discontinuous point.
          while next < nrow and tp >= table[next, 1] loop
            next := next + 1;
          end while;

          // Define next time event, if last table entry not reached
          if next < nrow then
            nextEventScaled := shiftTimeScaled + table[next, 1];
          end if;

          // Determine interpolation coefficients
          if next == 1 then
            next := 2;
          end if;
          next0 := next - 1;
          dt := table[next, 1] - table[next0, 1];
          if dt <= TimeEps*abs(table[next, 1]) then
            // Interpolation interval is not big enough, use "next" value
            a := 0;
            b := offset + table[next, columns];
          else
            a := (table[next, columns] - table[next0, columns])/dt;
            b := offset + table[next0, columns] - a*table[next0, 1];
          end if;
        end if;
        // Take into account shiftTimeScaled "a*(time - shiftTime) + b"
        b := b - a*shiftTimeScaled;
      end getInterpolationCoefficients;
    algorithm
      if noEvent(size(table, 1) > 1) then
        assert(not (table[1, 1] > 0.0 or table[1, 1] < 0.0), "The first point in time has to be set to 0, but is table[1,1] = " + String(table[1, 1]));
      end if;
      when {time >= pre(nextEvent),initial()} then
        (a,b,nextEventScaled,last) := getInterpolationCoefficients(
            table,
            offset,
            startTime/timeScale,
            timeScaled,
            last,
            100*Modelica.Constants.eps,
            shiftTime/timeScale);
        nextEvent := nextEventScaled*timeScale;
      end when;
    equation
      assert(size(table, 1) > 0, "No table values defined.");
      timeScaled = time/timeScale;
      y = a*timeScaled + b;
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-48,70},{2,-50}},
              lineColor={255,255,255},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-48,-50},{-48,70},{52,70},{52,-50},{-48,-50},{-48,-20},
                  {52,-20},{52,10},{-48,10},{-48,40},{52,40},{52,70},{2,70},{2,-51}}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="offset=%offset")}),
            Documentation(info="<html>
<p>
This block generates an output signal by <strong>linear interpolation</strong> in
a table. The time points and function values are stored in a matrix
<strong>table[i,j]</strong>, where the first column table[:,1] contains the
time points and the second column contains the data to be interpolated.
The table interpolation has the following properties:
</p>
<ul>
<li>The interpolation interval is found by a linear search where the interval used in the
    last call is used as start interval.</li>
<li>The time points need to be <strong>monotonically increasing</strong>.</li>
<li><strong>Discontinuities</strong> are allowed, by providing the same
    time point twice in the table.</li>
<li>Values <strong>outside</strong> of the table range, are computed by
    <strong>extrapolation</strong> through the last or first two points of the
    table.</li>
<li>If the table has only <strong>one row</strong>, no interpolation is performed and
    the function value is just returned independently of the actual time instant.</li>
<li>Via parameters <strong>shiftTime</strong> and <strong>offset</strong> the curve defined
    by the table can be shifted both in time and in the ordinate value.
    The time instants stored in the table are therefore <strong>relative</strong>
    to <strong>shiftTime</strong>.</li>
<li>If time &lt; startTime, no interpolation is performed and the offset
    is used as ordinate value for the output.</li>
<li>If the table has more than one row, the first point in time <strong>always</strong> has to be set to <strong>0</strong>, e.g.,
    <strong>table=[1,1;2,2]</strong> is <strong>illegal</strong>. If you want to
    shift the time table in time use the <strong>shiftTime</strong> parameter instead.</li>
<li>The table is implemented in a numerically sound way by
    generating <strong>time events</strong> at interval boundaries.
    This generates continuously differentiable values for the integrator.</li>
<li>Via parameter <strong>timeScale</strong> the first column of the table array can
    be scaled, e.g., if the table array is given in hours (instead of seconds)
    <strong>timeScale</strong> shall be set to 3600.</li>
</ul>
<p>
Example:
</p>
<blockquote><pre>
   table = [0, 0;
            1, 0;
            1, 1;
            2, 4;
            3, 9;
            4, 16];
If, e.g., time = 1.0, the output y =  0.0 (before event), 1.0 (after event)
    e.g., time = 1.5, the output y =  2.5,
    e.g., time = 2.0, the output y =  4.0,
    e.g., time = 5.0, the output y = 23.0 (i.e., extrapolation).
</pre></blockquote>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/TimeTable.png\"
     alt=\"TimeTable.png\">
</p>

</html>",   revisions="<html>
<h4>Release Notes</h4>
<ul>
<li><em>Oct. 21, 2002</em>
       by Christian Schweiger:<br>
       Corrected interface from
<blockquote><pre>
parameter Real table[:, :]=[0, 0; 1, 1; 2, 4];
</pre></blockquote>
       to
<blockquote><pre>
parameter Real table[:, <strong>2</strong>]=[0, 0; 1, 1; 2, 4];
</pre></blockquote>
       </li>
<li><em>Nov. 7, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized.</li>
</ul>
</html>"));
    end TimeTable;

    block CombiTimeTable
      "Table look-up with respect to time and linear/periodic extrapolation methods (data from matrix/file)"
      import Blocks.Tables.Internal;
      extends Blocks.Interfaces.MO(final nout=max([size(columns, 1); size(
            offset, 1)]));
      parameter Boolean tableOnFile=false
        "= true, if table is defined on file or in function usertab"
        annotation (Dialog(group="Table data definition"));
      parameter Real table[:, :] = fill(0.0, 0, 2)
        "Table matrix (time = first column; e.g., table=[0, 0; 1, 1; 2, 4])"
        annotation (Dialog(group="Table data definition",enable=not tableOnFile));
      parameter String tableName="NoName"
        "Table name on file or in function usertab (see docu)"
        annotation (Dialog(group="Table data definition",enable=tableOnFile));
      parameter String fileName="NoName" "File where matrix is stored"
        annotation (Dialog(
          group="Table data definition",
          enable=tableOnFile,
          loadSelector(filter="Text files (*.txt);;MATLAB MAT-files (*.mat)",
              caption="Open file in which table is present")));
      parameter Boolean verboseRead=true
        "= true, if info message that file is loading is to be printed"
        annotation (Dialog(group="Table data definition",enable=tableOnFile));
      parameter Integer columns[:]=2:size(table, 2)
        "Columns of table to be interpolated"
        annotation (Dialog(group="Table data interpretation",
        groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/CombiTimeTable.png"));
      parameter Blocks.Types.Smoothness smoothness=Blocks.Types.Smoothness.LinearSegments
        "Smoothness of table interpolation"
        annotation (Dialog(group="Table data interpretation"));
      parameter Blocks.Types.Extrapolation extrapolation=Blocks.Types.Extrapolation.LastTwoPoints
        "Extrapolation of data outside the definition range"
        annotation (Dialog(group="Table data interpretation"));
      parameter SI.Time timeScale(
        min=Modelica.Constants.eps)=1 "Time scale of first table column"
        annotation (Dialog(group="Table data interpretation"), Evaluate=true);
      parameter Real offset[:]={0} "Offsets of output signals"
        annotation (Dialog(group="Table data interpretation"));
      parameter SI.Time startTime=0
        "Output = offset for time < startTime"
        annotation (Dialog(group="Table data interpretation"));
      parameter SI.Time shiftTime=startTime
        "Shift time of first table column"
        annotation (Dialog(group="Table data interpretation"));
      parameter Blocks.Types.TimeEvents timeEvents=Blocks.Types.TimeEvents.Always
        "Time event handling of table interpolation" annotation (Dialog(group=
              "Table data interpretation", enable=smoothness == Blocks.Types.Smoothness.LinearSegments));
      parameter Boolean verboseExtrapolation=false
        "= true, if warning messages are to be printed if time is outside the table definition range"
        annotation (Dialog(group="Table data interpretation", enable=
              extrapolation == Blocks.Types.Extrapolation.LastTwoPoints or
              extrapolation == Blocks.Types.Extrapolation.HoldLastPoint));
      final parameter SI.Time t_min=t_minScaled*timeScale
        "Minimum abscissa value defined in table";
      final parameter SI.Time t_max=t_maxScaled*timeScale
        "Maximum abscissa value defined in table";
      final parameter Real t_minScaled=Internal.getTimeTableTmin(tableID)
        "Minimum (scaled) abscissa value defined in table";
      final parameter Real t_maxScaled=Internal.getTimeTableTmax(tableID)
        "Maximum (scaled) abscissa value defined in table";
    protected
      final parameter Real p_offset[nout]=(if size(offset, 1) == 1 then ones(nout)*offset[1] else offset)
        "Offsets of output signals";
      parameter Blocks.Types.ExternalCombiTimeTable tableID=
          Blocks.Types.ExternalCombiTimeTable(
              if tableOnFile then tableName else "NoName",
              if tableOnFile and fileName <> "NoName" and not
            Modelica.Utilities.Strings.isEmpty(fileName) then fileName else
            "NoName",
              table,
              startTime/timeScale,
              columns,
              smoothness,
              extrapolation,
              shiftTime/timeScale,
              if smoothness == Blocks.Types.Smoothness.LinearSegments then
            timeEvents elseif smoothness == Blocks.Types.Smoothness.ConstantSegments
             then Blocks.Types.TimeEvents.Always else Blocks.Types.TimeEvents.NoTimeEvents,
              if tableOnFile then verboseRead else false)
        "External table object";
      discrete SI.Time nextTimeEvent(start=0, fixed=true)
        "Next time event instant";
      discrete Real nextTimeEventScaled(start=0, fixed=true)
        "Next scaled time event instant";
      Real timeScaled "Scaled time";
    equation
      if tableOnFile then
        assert(tableName <> "NoName",
          "tableOnFile = true and no table name given");
      else
        assert(size(table, 1) > 0 and size(table, 2) > 0,
          "tableOnFile = false and parameter table is an empty matrix");
      end if;

      if verboseExtrapolation and (extrapolation == Blocks.Types.Extrapolation.LastTwoPoints
           or extrapolation == Blocks.Types.Extrapolation.HoldLastPoint) then
        assert(noEvent(time >= t_min), "
Extrapolation warning: Time (="   + String(time) + ") must be greater or equal
than the minimum abscissa value t_min (="   + String(t_min) + ") defined in the table.
",   level=AssertionLevel.warning);
        assert(noEvent(time <= t_max), "
Extrapolation warning: Time (="   + String(time) + ") must be less or equal
than the maximum abscissa value t_max (="   + String(t_max) + ") defined in the table.
",   level=AssertionLevel.warning);
      end if;

      timeScaled = time/timeScale;
      when {time >= pre(nextTimeEvent), initial()} then
        nextTimeEventScaled = Internal.getNextTimeEvent(tableID, timeScaled);
        nextTimeEvent = if nextTimeEventScaled < Modelica.Constants.inf then nextTimeEventScaled*timeScale else Modelica.Constants.inf;
      end when;
      if smoothness == Blocks.Types.Smoothness.ConstantSegments then
        for i in 1:nout loop
          y[i] = p_offset[i] + Internal.getTimeTableValueNoDer(tableID, i, timeScaled, nextTimeEventScaled, pre(nextTimeEventScaled));
        end for;
      elseif smoothness == Blocks.Types.Smoothness.LinearSegments then
        for i in 1:nout loop
          y[i] = p_offset[i] + Internal.getTimeTableValueNoDer2(tableID, i, timeScaled, nextTimeEventScaled, pre(nextTimeEventScaled));
        end for;
      else
        for i in 1:nout loop
          y[i] = p_offset[i] + Internal.getTimeTableValue(tableID, i, timeScaled, nextTimeEventScaled, pre(nextTimeEventScaled));
        end for;
      end if;
      annotation (
        Documentation(info="<html>
<p>
This block generates an output signal y[:] by <strong>constant</strong>,
<strong>linear</strong> or <strong>cubic Hermite spline interpolation</strong>
in a table. The time points and function values are stored in a matrix
<strong>table[i,j]</strong>, where the first column table[:,1] contains the
time points and the other columns contain the data to be interpolated.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/CombiTimeTable.png\"
     alt=\"CombiTimeTable.png\">
</p>

<p>
Via parameter <strong>columns</strong> it can be defined which columns of the
table are interpolated. If, e.g., columns={2,4}, it is assumed that
2 output signals are present and that the first output is computed
by interpolation of column 2 and the second output is computed
by interpolation of column 4 of the table matrix.
The table interpolation has the following properties:
</p>
<ul>
<li>The interpolation interval is found by a binary search where the interval used in the
    last call is used as start interval.</li>
<li>The time points need to be <strong>strictly increasing</strong> for cubic Hermite
    spline interpolation, otherwise <strong>monotonically increasing</strong>.</li>
<li><strong>Discontinuities</strong> are allowed for (constant or) linear interpolation,
    by providing the same time point twice in the table.</li>
<li>Via parameter <strong>smoothness</strong> it is defined how the data is interpolated:
<blockquote><pre>
smoothness = 1: Linear interpolation
           = 2: Akima interpolation: Smooth interpolation by cubic Hermite
                splines such that der(y) is continuous, also if extrapolated.
           = 3: Constant segments
           = 4: Fritsch-Butland interpolation: Smooth interpolation by cubic
                Hermite splines such that y preserves the monotonicity and
                der(y) is continuous, also if extrapolated.
           = 5: Steffen interpolation: Smooth interpolation by cubic Hermite
                splines such that y preserves the monotonicity and der(y)
                is continuous, also if extrapolated.
           = 6: Modified Akima interpolation: Smooth interpolation by cubic
                Hermite splines such that der(y) is continuous, also if
                extrapolated. Additionally, overshoots and edge cases of the
                original Akima interpolation method are avoided.
</pre></blockquote></li>
<li>First and second <strong>derivatives</strong> are provided, with exception of the following two smoothness options.
<ol>
<li>No derivatives are provided for interpolation by constant segments.</li>
<li>No second derivative is provided for linear interpolation.<br>There is a design inconsistency, that it is possible
to model a signal consisting of constant segments using linear interpolation and duplicated sample points.
In contrast to interpolation by constant segments, the first derivative is provided as zero.</li>
</ol></li>
<li>Values <strong>outside</strong> of the table range, are computed by
    extrapolation according to the setting of parameter <strong>extrapolation</strong>:
<blockquote><pre>
extrapolation = 1: Hold the first or last value of the table,
                   if outside of the table scope.
              = 2: Extrapolate by using the derivative at the first/last table
                   points if outside of the table scope.
                   (If smoothness is LinearSegments or ConstantSegments
                   this means to extrapolate linearly through the first/last
                   two table points.).
              = 3: Periodically repeat the table data (periodical function).
              = 4: No extrapolation, i.e. extrapolation triggers an error
</pre></blockquote></li>
<li>If the table has only <strong>one row</strong>, no interpolation is performed and
    the table values of this row are just returned.</li>
<li>Via parameters <strong>shiftTime</strong> and <strong>offset</strong> the curve defined
    by the table can be shifted both in time and in the ordinate value.
    The time instants stored in the table are therefore <strong>relative</strong>
    to <strong>shiftTime</strong>.</li>
<li>If time &lt; startTime, no interpolation is performed and the offset
    is used as ordinate value for all outputs.</li>
<li>The table is implemented in a numerically sound way by
    generating <strong>time events</strong> at interval boundaries, in case of
    interpolation by linear segments.
    This generates continuously differentiable values for the integrator.
    Via parameter <strong>timeEvents</strong> it is defined how the time events are generated:
<blockquote><pre>
timeEvents = 1: Always generate time events at interval boundaries
           = 2: Generate time events at discontinuities (defined by duplicated sample points)
           = 3: No time events at interval boundaries
</pre></blockquote>
    For interpolation by constant segments time events are always generated at interval boundaries.
    For smooth interpolation by cubic Hermite splines no time events are generated at interval boundaries.</li>
<li>Via parameter <strong>timeScale</strong> the first column of the table array can
    be scaled, e.g., if the table array is given in hours (instead of seconds)
    <strong>timeScale</strong> shall be set to 3600.</li>
<li>For special applications it is sometimes needed to know the minimum
    and maximum time instant defined in the table as a parameter. For this
    reason parameters <strong>t_min</strong>/<strong>t_minScaled</strong> and
    <strong>t_max</strong>/<strong>t_maxScaled</strong> are provided and can be
    accessed from the outside of the table object. Whereas <strong>t_min</strong> and
    <strong>t_max</strong> define the scaled abscissa values (using parameter
    <strong>timeScale</strong>) in SI.Time, <strong>t_minScaled</strong> and
    <strong>t_maxScaled</strong> define the unitless original abscissa values of
    the table.</li>
</ul>
<p>
Example:
</p>
<blockquote><pre>
table = [0, 0;
         1, 0;
         1, 1;
         2, 4;
         3, 9;
         4, 16];
extrapolation = 2 (default), timeEvents = 2
If, e.g., time = 1.0, the output y =  0.0 (before event), 1.0 (after event)
    e.g., time = 1.5, the output y =  2.5,
    e.g., time = 2.0, the output y =  4.0,
    e.g., time = 5.0, the output y = 23.0 (i.e., extrapolation via last 2 points).
</pre></blockquote>
<p>
The table matrix can be defined in the following ways:
</p>
<ol>
<li>Explicitly supplied as <strong>parameter matrix</strong> \"table\",
    and the other parameters have the following values:
<blockquote><pre>
tableName is \"NoName\" or has only blanks,
fileName  is \"NoName\" or has only blanks.
</pre></blockquote></li>
<li><strong>Read</strong> from a <strong>file</strong> \"fileName\" where the matrix is stored as
    \"tableName\". Both text and MATLAB MAT-file format is possible.
    (The text format is described below).
    The MAT-file format comes in four different versions: v4, v6, v7 and v7.3.
    The library supports at least v4, v6 and v7 whereas v7.3 is optional.
    It is most convenient to generate the MAT-file from FreeMat or MATLAB&reg;
    by command
<blockquote><pre>
save tables.mat tab1 tab2 tab3
</pre></blockquote>
    or Scilab by command
<blockquote><pre>
savematfile tables.mat tab1 tab2 tab3
</pre></blockquote>
    when the three tables tab1, tab2, tab3 should be used from the model.<br>
    Note, a fileName can be defined as URI by using the helper function
    <a href=\"modelica://Modelica.Utilities.Files.loadResource\">loadResource</a>.</li>
<li>Statically stored in function \"usertab\" in file \"usertab.c\".
    The matrix is identified by \"tableName\". Parameter
    fileName = \"NoName\" or has only blanks. Row-wise storage is always to be
    preferred as otherwise the table is reallocated and transposed.</li>
</ol>
<p>
When the constant \"NO_FILE_SYSTEM\" is defined, all file I/O related parts of the
source code are removed by the C-preprocessor, such that no access to files takes place.
</p>
<p>
If tables are read from a text file, the file needs to have the
following structure (\"-----\" is not part of the file content):
</p>
<blockquote><pre>
-----------------------------------------------------
#1
double tab1(6,2)   # comment line
  0   0
  1   0
  1   1
  2   4
  3   9
  4  16
double tab2(6,2)   # another comment line
  0   0
  2   0
  2   2
  4   8
  6  18
  8  32
-----------------------------------------------------
</pre></blockquote>
<p>
Note, that the first two characters in the file need to be
\"#1\" (a line comment defining the version number of the file format).
Afterwards, the corresponding matrix has to be declared
with type (= \"double\" or \"float\"), name and actual dimensions.
Finally, in successive rows of the file, the elements of the matrix
have to be given. The elements have to be provided as a sequence of
numbers in row-wise order (therefore a matrix row can span several
lines in the file and need not start at the beginning of a line).
Numbers have to be given according to C syntax (such as 2.3, -2, +2.e4).
Number separators are spaces, tab (\\t), comma (,), or semicolon (;).
Several matrices may be defined one after another. Line comments start
with the hash symbol (#) and can appear everywhere.
Text files should either be ASCII or UTF-8 encoded, where UTF-8 encoded strings are only allowed in line comments and an optional UTF-8 BOM at the start of the text file is ignored.
Other characters, like trailing non comments, are not allowed in the file.
</p>
<p>
MATLAB is a registered trademark of The MathWorks, Inc.
</p>
</html>",   revisions="<html>
<p><strong>Release Notes:</strong></p>
<ul>
<li><em>April 09, 2013</em>
       by Thomas Beutlich:<br>
       Implemented as external object.</li>
<li><em>March 31, 2001</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Used CombiTableTime as a basis and added the
       arguments <strong>extrapolation, columns, startTime</strong>.
       This allows periodic function definitions.</li>
</ul>
</html>"),
        Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Polygon(lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid,
          points={{-80.0,90.0},{-88.0,68.0},{-72.0,68.0},{-80.0,90.0}}),
        Line(points={{-80.0,68.0},{-80.0,-80.0}},
          color={192,192,192}),
        Line(points={{-90.0,-70.0},{82.0,-70.0}},
          color={192,192,192}),
        Polygon(lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid,
          points={{90.0,-70.0},{68.0,-62.0},{68.0,-78.0},{90.0,-70.0}}),
        Rectangle(lineColor={255,255,255},
          fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-48.0,-50.0},{2.0,70.0}}),
        Line(points={{-48.0,-50.0},{-48.0,70.0},{52.0,70.0},{52.0,-50.0},{-48.0,-50.0},{-48.0,-20.0},{52.0,-20.0},{52.0,10.0},{-48.0,10.0},{-48.0,40.0},{52.0,40.0},{52.0,70.0},{2.0,70.0},{2.0,-51.0}})}));
    end CombiTimeTable;

    block BooleanConstant "Generate constant signal of type Boolean"
      parameter Boolean k=true "Constant output value"
      annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/BooleanConstant.png"));
      extends Interfaces.partialBooleanSource;

    equation
      y = k;
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={Line(points={{-80,0},{80,0}}),
              Text(
              extent={{-150,-140},{150,-110}},
              textString="%k")}),
          Documentation(info="<html>
<p>
The Boolean output y is a constant signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/BooleanConstant.png\"
     alt=\"BooleanConstant.png\">
</p>
</html>"));
    end BooleanConstant;

    block BooleanStep "Generate step signal of type Boolean"
      parameter SI.Time startTime=0 "Time instant of step start"
       annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/BooleanStep.png"));
      parameter Boolean startValue=false "Output before startTime";

      extends Interfaces.partialBooleanSource;
    equation
      y = if time >= startTime then not startValue else startValue;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(
              visible=not startValue,
              points={{-80,-70},{0,-70},{0,50},{80,50}}),
            Line(
              visible=startValue,
              points={{-80,50},{0,50},{0,-70},{68,-70}}),
            Text(
              extent={{-150,-140},{150,-110}},
              textString="%startTime")}),
        Documentation(info="<html>
<p>
The Boolean output y is a step signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/BooleanStep.png\"
     alt=\"BooleanStep.png\">
</p>
</html>"));
    end BooleanStep;

    block BooleanPulse "Generate pulse signal of type Boolean"

      parameter Real width(
        final min=Modelica.Constants.small,
        final max=100) = 50 "Width of pulse in % of period"
        annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/BooleanPulse.png"));
      parameter SI.Time period(final min=Modelica.Constants.small,
          start=1) "Time for one period";
      parameter SI.Time startTime=0 "Time instant of first pulse";
      extends Blocks.Interfaces.partialBooleanSource;

    protected
      parameter SI.Time Twidth=period*width/100
        "Width of one pulse" annotation (HideResult=true);
      discrete SI.Time pulseStart "Start time of pulse"
        annotation (HideResult=true);
    initial equation
      pulseStart = startTime;
    equation
      when sample(startTime, period) then
        pulseStart = time;
      end when;
      y = time >= pulseStart and time < pulseStart + Twidth;
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={Text(
              extent={{-150,-140},{150,-110}},
              textString="%period"), Line(points={{-80,-70},{-40,-70},{-40,44},{0,
                  44},{0,-70},{40,-70},{40,44},{79,44}})}),
          Documentation(info="<html>
<p>
The Boolean output y is a pulse signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/Pulse.png\"
     alt=\"Pulse.png\">
</p>
</html>"));
    end BooleanPulse;

    block SampleTrigger "Generate sample trigger signal"
      parameter SI.Time period(final min=Modelica.Constants.small,
          start=0.01) "Sample period"
        annotation(Dialog(groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/SampleTrigger.png"));
      parameter SI.Time startTime=0
        "Time instant of first sample trigger";
      extends Interfaces.partialBooleanSource;

    equation
      y = sample(startTime, period);
      annotation (
        Icon(coordinateSystem(
            preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}), graphics={
            Line(points={{-60,-70},{-60,70}}),
            Line(points={{-20,-70},{-20,70}}),
            Line(points={{20,-70},{20,70}}),
            Line(points={{60,-70},{60,70}}),
            Text(
              extent={{-150,-140},{150,-110}},
              textString="%period")}),
          Documentation(info="<html>
<p>
The Boolean output y is a trigger signal where the output y is only <strong>true</strong>
at sample times (defined by parameter <strong>period</strong>) and is otherwise
<strong>false</strong>.
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/SampleTrigger.png\"
     alt=\"SampleTrigger.png\">
</p>
</html>"));
    end SampleTrigger;

    block BooleanTable
      "Generate a Boolean output signal based on a vector of time instants"

      parameter SI.Time table[:]={0,1}
        "Vector of time points. At every time point, the output y gets its opposite value (e.g., table={0,1})" annotation(Dialog(group="Table data definition"));
      parameter Boolean startValue=false
        "Start value of y. At time = table[1], y changes to 'not startValue'" annotation(Dialog(group="Table data interpretation",
        groupImage="modelica://Modelica/Resources/Images/Blocks/Sources/BooleanTable.png"));
      parameter Blocks.Types.Extrapolation extrapolation=Blocks.Types.Extrapolation.HoldLastPoint
        "Extrapolation of data outside the definition range"
        annotation (Dialog(group="Table data interpretation"));
      parameter SI.Time startTime=-Modelica.Constants.inf
        "Output = false for time < startTime" annotation(Dialog(group="Table data interpretation"));
      parameter SI.Time shiftTime=0
        "Shift time of table" annotation(Dialog(group="Table data interpretation"));

      extends Interfaces.partialBooleanSO;

      CombiTimeTable combiTimeTable(
        final table=if n > 0 then if startValue then [table[1], 1.0; table, {mod(i + 1, 2.0) for i in 1:n}] else [table[1], 0.0; table, {mod(i, 2.0) for i in 1:n}] else if startValue then [0.0, 1.0] else [0.0, 0.0],
        final smoothness=Blocks.Types.Smoothness.ConstantSegments,
        final columns={2},
        final extrapolation=extrapolation,
        final startTime=startTime,
        final shiftTime=shiftTime) annotation(Placement(transformation(extent={{-30,-10},{-10,10}})));
      Blocks.Math.RealToBoolean realToBoolean
        annotation (Placement(transformation(extent={{10,-10},{30,10}})));

    protected
        function isValidTable "Check if table is valid"
          extends Modelica.Icons.Function;
          input Real table[:] "Vector of time instants";
      protected
          Integer n=size(table, 1) "Number of table points";
        algorithm
          if n > 0 then
            // Check whether time values are strict monotonically increasing
            for i in 2:n loop
              assert(table[i] > table[i-1],
                "Time values of table not strict monotonically increasing: table["
                 + String(i - 1) + "] = " + String(table[i - 1]) + ", table[" +
                String(i) + "] = " + String(table[i]));
            end for;
          end if;
        end isValidTable;

        parameter Integer n=size(table, 1) "Number of table points";
    initial algorithm
        isValidTable(table);
    equation
      assert(extrapolation <> Blocks.Types.Extrapolation.LastTwoPoints,
        "Unsuitable extrapolation setting.");
        connect(combiTimeTable.y[1], realToBoolean.u) annotation(Line(points={{-9,0},{8,0}}, color={0,0,127}));
        connect(realToBoolean.y, y) annotation(Line(points={{31,0},{110,0},{110,0}}, color={255,127,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={Polygon(
              points={{-80,88},{-88,66},{-72,66},{-80,88}},
              lineColor={255,0,255},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,66},{-80,-82}}, color={255,0,255}),
            Line(points={{-90,-70},{72,-70}}, color={255,0,255}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={255,0,255},
              fillColor={255,0,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-18,70},{32,-50}},
              lineColor={255,255,255},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid), Line(points={{-18,-50},{-18,70},{32,
                  70},{32,-50},{-18,-50},{-18,-20},{32,-20},{32,10},{-18,10},{-18,
                  40},{32,40},{32,70},{32,70},{32,-51}})}),
        Documentation(info="<html>
<p>
The Boolean output y is a signal defined by parameter vector <strong>table</strong>.
In the vector time points are stored.
The table interpolation has the following properties:
</p>

<ul>
<li>At every time point, the output y
    changes its value to the negated value of the previous one.</li>
<li>Values <strong>outside</strong> of the table range, are computed by
    extrapolation according to the setting of parameter <strong>extrapolation</strong>:
<blockquote><pre>
extrapolation = 1: Hold the <strong>startValue</strong> or last value of the table,
                   if outside of the table scope.
              = 2: Extrapolate by using the derivative at the first/last table
                   points if outside of the table scope.
                   (This setting is not suitable and triggers an assert.)
              = 3: Periodically repeat the table data (periodical function).
              = 4: No extrapolation, i.e. extrapolation triggers an error
</pre></blockquote></li>
<li>Via parameter <strong>shiftTime</strong> the curve defined by the table can be shifted
    in time.
    The time instants stored in the table are therefore <strong>relative</strong>
    to <strong>shiftTime</strong>.</li>
<li>If time &lt; startTime, no interpolation is performed and <strong>false</strong>
    is used as ordinate value for the output.</li>
</ul>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/BooleanTable.png\"
     alt=\"BooleanTable.png\">
</p>

<p>
The precise semantics is:
</p>

<blockquote><pre>
<strong>if</strong> size(table,1) == 0 <strong>then</strong>
   y = startValue;
<strong>else</strong>
   //            time &lt; table[1]: y = startValue
   // table[1] &le; time &lt; table[2]: y = not startValue
   // table[2] &le; time &lt; table[3]: y = startValue
   // table[3] &le; time &lt; table[4]: y = not startValue
   // ...
<strong>end if</strong>;
</pre></blockquote>
</html>"));
    end BooleanTable;

    block RadioButtonSource "Boolean signal source that mimics a radio button"

      parameter SI.Time buttonTimeTable[:]={0,1}
        "Time instants where button is pressed";
      input Boolean reset[:]={false}
        "Reset button to false, if an element of reset becomes true"
        annotation (Dialog(group="Time varying expressions"));

      Blocks.Interfaces.BooleanOutput on(start=false, fixed=true)
        annotation (Placement(transformation(extent={{100,-15},{130,15}})));
    protected
      Blocks.Sources.BooleanTable table(table=buttonTimeTable, y(start=false,
            fixed=true));
      parameter Integer nReset=size(reset, 1);
      Boolean pre_reset[nReset];
    initial equation
      pre(pre_reset) = fill(false, nReset);
      pre(reset) = fill(false, nReset);
    algorithm
      pre_reset := pre(reset);
      when pre_reset then
        on := false;
      end when;

      when change(table.y) then
        on := true;
      end when;

      annotation (Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{-100,-100},{100,100}},
            initialScale=0.06), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              borderPattern=BorderPattern.Raised,
              fillColor=DynamicSelect({192,192,192}, if on then {0,255,0} else {192,192,192}),
              fillPattern=FillPattern.Solid,
              lineColor={128,128,128}), Text(
              extent={{-300,110},{300,175}},
              textColor={0,0,255},
              textString="%name")},
            interaction={OnMouseDownSetBoolean(on, true)}), Documentation(info="<html>
<p>
Boolean signal source that mimics a radio button:
Via a table, a radio button is pressed (i.e., the output 'on' is set to true) and is reset when an element of the Boolean vector
'reset' becomes true. If both appear at the same time instant, setting
the button according to the table has a higher priority as resetting
the button. Example:
</p>

<blockquote><pre>
RadioButtonSource start(buttonTimeTable={1,3}, reset={stop.on});
RadioButtonSource stop (buttonTimeTable={2,4}, reset={start.on});
</pre></blockquote>

<p>
The \"start\" button is pressed at time=1 s and time=3 s,
whereas the \"stop\" button is pressed at time=2 s and time=4 s.
This gives the following result:
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/RadioButtonSource.png\"
     alt=\"RadioButtonSource.png\">
</blockquote>

<p>
This example is also available in
<a href=\"modelica://Modelica.Blocks.Examples.Interaction1\">Modelica.Blocks.Examples.Interaction1</a>
</p>

</html>"));
    end RadioButtonSource;

    block IntegerConstant "Generate constant signal of type Integer"
      parameter Integer k(start=1) "Constant output value";
      extends Interfaces.IntegerSO;

    equation
      y = k;
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,0},{80,0}}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="k=%k")}),
        Documentation(info="<html>
<p>
The Integer output y is a constant signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/IntegerConstant.png\"
     alt=\"IntegerConstant.png\">
</p>
</html>"));
    end IntegerConstant;

    block IntegerStep "Generate step signal of type Integer"
      parameter Integer height=1 "Height of step";
      extends Interfaces.IntegerSignalSource;
    equation
      y = offset + (if time < startTime then 0 else height);
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Line(points={{-80,68},{-80,-80}}, color={192,192,192}),
            Polygon(
              points={{-80,90},{-88,68},{-72,68},{-80,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-70},{82,-70}}, color={192,192,192}),
            Polygon(
              points={{90,-70},{68,-62},{68,-78},{90,-70}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-70},{0,-70},{0,50},{80,50}}),
            Text(
              extent={{-150,-150},{150,-110}},
              textString="startTime=%startTime")}),
        Documentation(info="<html>
<p>
The Integer output y is a step signal:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/IntegerStep.png\"
     alt=\"IntegerStep.png\">
</p>
</html>"));
    end IntegerStep;

    block IntegerTable
      "Generate an Integer output signal based on a table matrix with [time, yi] values"

      parameter Real table[:, 2]=fill(0,0,2) "Table matrix (first column: time; second column: y)" annotation(Dialog(group="Table data definition"));
      parameter Blocks.Types.Extrapolation extrapolation=Blocks.Types.Extrapolation.HoldLastPoint
        "Extrapolation of data outside the definition range"
        annotation (Dialog(group="Table data interpretation"));
      parameter SI.Time startTime=-Modelica.Constants.inf
        "Output = 0 for time < startTime" annotation(Dialog(group="Table data interpretation"));
      parameter SI.Time shiftTime=0
        "Shift time of first table column" annotation(Dialog(group="Table data interpretation"));

      extends Interfaces.IntegerSO;

      CombiTimeTable combiTimeTable(
        final table=table,
        final smoothness=Blocks.Types.Smoothness.ConstantSegments,
        final columns={2},
        final extrapolation=extrapolation,
        final startTime=startTime,
        final shiftTime=shiftTime) annotation(Placement(transformation(extent={{-30,-10},{-10,10}})));
      Blocks.Math.RealToInteger realToInteger
        annotation (Placement(transformation(extent={{10,-10},{30,10}})));

    protected
        function isValidTable "Check if table is valid"
          extends Modelica.Icons.Function;
          input Real table[:, 2] "Table matrix";
      protected
          SI.Time t_last;
          Integer n=size(table, 1) "Number of table points";
        algorithm
          if n > 0 then
            // Check whether time values are strict monotonically increasing
            t_last := table[1, 1];
            for i in 2:n loop
              assert(table[i, 1] > t_last,
                "Time values of table not strict monotonically increasing: table["
                 + String(i - 1) + ",1] = " + String(table[i - 1, 1]) + "table[" +
                String(i) + ",1] = " + String(table[i, 1]));
            end for;

            // Check that all values in the second column are Integer values
            for i in 1:n loop
              assert(rem(table[i, 2], 1) == 0.0,
                "Table value is not an Integer: table[" + String(i) + ",2] = " +
                String(table[i, 2]));
            end for;
          end if;
        end isValidTable;

        parameter Integer n=size(table, 1) "Number of table points";
    initial algorithm
        isValidTable(table);
    equation
        assert(n > 0, "No table values defined.");
      assert(extrapolation <> Blocks.Types.Extrapolation.LastTwoPoints,
        "Unsuitable extrapolation setting.");
        connect(combiTimeTable.y[1], realToInteger.u) annotation(Line(points={{-9,0},{8,0}}, color={0,0,127}));
        connect(realToInteger.y, y) annotation(Line(points={{31,0},{110,0},{110,0}}, color={255,127,0}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
          graphics={
            Line(points={{-80,64},{-80,-84}}, color={192,192,192}),
            Polygon(
              points={{-80,86},{-88,64},{-72,64},{-80,86}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,-74},{82,-74}}, color={192,192,192}),
            Polygon(
              points={{90,-74},{68,-66},{68,-82},{90,-74}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-46,68},{4,-52}},
              lineColor={255,255,255},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-46,-52},{-46,68},{54,68},{54,-52},{-46,-52},{-46,-22},
                  {54,-22},{54,8},{-46,8},{-46,38},{54,38},{54,68},{4,68},{4,-53}})}),
        Documentation(info="<html>

<p>
This block generates an Integer output signal by using a table.
The time points and y-values are stored in a matrix
<strong>table[i,j]</strong>, where the first column table[:,1] contains the
Real time points and the second column contains the Integer value of the
output y at this time point.
The table interpolation has the following properties:
</p>

<ul>
<li>An assert is triggered, if no table values are provided, if the
    time points are not strict monotonically increasing, or if
    the second column of the table matrix does not contain Integer values.</li>
<li>Values <strong>outside</strong> of the table range, are computed by
    extrapolation according to the setting of parameter <strong>extrapolation</strong>:
<blockquote><pre>
extrapolation = 1: Hold the first or last value of the table,
                   if outside of the table scope.
              = 2: Extrapolate by using the derivative at the first/last table
                   points if outside of the table scope.
                   (This setting is not suitable and triggers an assert.)
              = 3: Periodically repeat the table data (periodical function).
              = 4: No extrapolation, i.e. extrapolation triggers an error
</pre></blockquote></li>
<li>If the table has only <strong>one row</strong>, no interpolation is performed and
    the table values of this row are just returned.</li>
<li>Via parameter <strong>shiftTime</strong> the curve defined by the table can be shifted
    in time.
    The time instants stored in the table are therefore <strong>relative</strong>
    to <strong>shiftTime</strong>.</li>
<li>If time &lt; startTime, no interpolation is performed and zero
    is used as ordinate value for the output.</li>
</ul>

<p>
Example:
</p>
<blockquote><pre>
table = [  0, 1;
           1, 4;
         1.5, 5;
           2, 6];
</pre></blockquote>
<p>
results in the following output:
</p>

<blockquote><p>
<img src=\"modelica://Modelica/Resources/Images/Blocks/Sources/IntegerTable.png\"
     alt=\"IntegerTable.png\">
</p></blockquote>

</html>"));
    end IntegerTable;
    annotation (Documentation(info="<html>
<p>
This package contains <strong>source</strong> components, i.e., blocks which
have only output signals. These blocks are used as signal generators
for Real, Integer and Boolean signals.
</p>

<p>
All Real source signals (with the exception of the Constant source)
have at least the following two parameters:
</p>

<table border=\"1\" cellspacing=\"0\" cellpadding=\"2\">
  <tr><td><strong>offset</strong></td>
      <td>Value which is added to the signal</td>
  </tr>
  <tr><td><strong>startTime</strong></td>
      <td>Start time of signal. For time &lt; startTime,
                the output y is set to offset.</td>
  </tr>
</table>

<p>
The <strong>offset</strong> parameter is especially useful in order to shift
the corresponding source, such that at initial time the system
is stationary. To determine the corresponding value of offset,
usually requires a trimming calculation.
</p>
</html>",   revisions="<html>
<ul>
<li><em>October 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       Integer sources added. Step, TimeTable and BooleanStep slightly changed.</li>
<li><em>Nov. 8, 1999</em>
       by <a href=\"mailto:christoph@clauss-it.com\">Christoph Clau&szlig;</a>,
       <a href=\"mailto:Andre.Schneider@eas.iis.fraunhofer.de\">Andre.Schneider@eas.iis.fraunhofer.de</a>,
       <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       New sources: Exponentials, TimeTable. Trapezoid slightly enhanced
       (nperiod=-1 is an infinite number of periods).</li>
<li><em>Oct. 31, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       <a href=\"mailto:christoph@clauss-it.com\">Christoph Clau&szlig;</a>,
       <a href=\"mailto:Andre.Schneider@eas.iis.fraunhofer.de\">Andre.Schneider@eas.iis.fraunhofer.de</a>,
       All sources vectorized. New sources: ExpSine, Trapezoid,
       BooleanConstant, BooleanStep, BooleanPulse, SampleTrigger.
       Improved documentation, especially detailed description of
       signals in diagram layer.</li>
<li><em>June 29, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"));
  end Sources;

  package Tables
    "Library of blocks to interpolate in one and two-dimensional tables"
    extends Modelica.Icons.Package;
    block CombiTable1Ds
      "Table look-up in one dimension (matrix/file) with one input and n outputs"
      extends Blocks.Interfaces.SIMO(final nout=size(columns, 1));
      parameter Boolean tableOnFile=false
        "= true, if table is defined on file or in function usertab"
        annotation (Dialog(group="Table data definition"));
      parameter Real table[:, :] = fill(0.0, 0, 2)
        "Table matrix (grid = first column; e.g., table=[0, 0; 1, 1; 2, 4])"
        annotation (Dialog(group="Table data definition",enable=not tableOnFile));
      parameter String tableName="NoName"
        "Table name on file or in function usertab (see docu)"
        annotation (Dialog(group="Table data definition",enable=tableOnFile));
      parameter String fileName="NoName" "File where matrix is stored"
        annotation (Dialog(
          group="Table data definition",
          enable=tableOnFile,
          loadSelector(filter="Text files (*.txt);;MATLAB MAT-files (*.mat)",
              caption="Open file in which table is present")));
      parameter Boolean verboseRead=true
        "= true, if info message that file is loading is to be printed"
        annotation (Dialog(group="Table data definition",enable=tableOnFile));
      parameter Integer columns[:]=2:size(table, 2)
        "Columns of table to be interpolated"
        annotation (Dialog(group="Table data interpretation"));
      parameter Blocks.Types.Smoothness smoothness=Blocks.Types.Smoothness.LinearSegments
        "Smoothness of table interpolation"
        annotation (Dialog(group="Table data interpretation"));
      parameter Blocks.Types.Extrapolation extrapolation=Blocks.Types.Extrapolation.LastTwoPoints
        "Extrapolation of data outside the definition range"
        annotation (Dialog(group="Table data interpretation"));
      parameter Boolean verboseExtrapolation=false
        "= true, if warning messages are to be printed if table input is outside the definition range"
        annotation (Dialog(group="Table data interpretation", enable=
              extrapolation == Blocks.Types.Extrapolation.LastTwoPoints or
              extrapolation == Blocks.Types.Extrapolation.HoldLastPoint));
      final parameter Real u_min=Internal.getTable1DAbscissaUmin(tableID)
        "Minimum abscissa value defined in table";
      final parameter Real u_max=Internal.getTable1DAbscissaUmax(tableID)
        "Maximum abscissa value defined in table";
    protected
      parameter Blocks.Types.ExternalCombiTable1D tableID=
          Blocks.Types.ExternalCombiTable1D(
              if tableOnFile then tableName else "NoName",
              if tableOnFile and fileName <> "NoName" and not
            Modelica.Utilities.Strings.isEmpty(fileName) then fileName else
            "NoName",
              table,
              columns,
              smoothness,
              extrapolation,
              if tableOnFile then verboseRead else false)
        "External table object";
    equation
      if tableOnFile then
        assert(tableName <> "NoName",
          "tableOnFile = true and no table name given");
      else
        assert(size(table, 1) > 0 and size(table, 2) > 0,
          "tableOnFile = false and parameter table is an empty matrix");
      end if;

      if verboseExtrapolation and (extrapolation == Blocks.Types.Extrapolation.LastTwoPoints
           or extrapolation == Blocks.Types.Extrapolation.HoldLastPoint) then
        assert(noEvent(u >= u_min), "
Extrapolation warning: The value u (="   + String(u) + ") must be greater or equal
than the minimum abscissa value u_min (="   + String(u_min) + ") defined in the table.
",   level=AssertionLevel.warning);
        assert(noEvent(u <= u_max), "
Extrapolation warning: The value u (="   + String(u) + ") must be less or equal
than the maximum abscissa value u_max (="   + String(u_max) + ") defined in the table.
",   level=AssertionLevel.warning);
      end if;

      if smoothness == Blocks.Types.Smoothness.ConstantSegments then
        for i in 1:nout loop
          y[i] = Internal.getTable1DValueNoDer(tableID, i, u);
        end for;
      elseif smoothness == Blocks.Types.Smoothness.LinearSegments then
        for i in 1:nout loop
          y[i] = Internal.getTable1DValueNoDer2(tableID, i, u);
        end for;
      else
        for i in 1:nout loop
          y[i] = Internal.getTable1DValue(tableID, i, u);
        end for;
      end if;
      annotation (
        Documentation(info="<html>
<p>
<strong>Univariate constant</strong>, <strong>linear</strong> or <strong>cubic Hermite
spline interpolation</strong> in <strong>one</strong> dimension of a
<strong>table</strong>.
Via parameter <strong>columns</strong> it can be defined how many columns of the
table are interpolated. If, e.g., columns={2,4}, it is assumed that
2 output signals are present and that the first output interpolates
via column 2 and the second output interpolates via column 4 of the
table matrix.
</p>
<p>
The grid points and function values are stored in a matrix \"table[i,j]\",
where the first column \"table[:,1]\" contains the grid points and the
other columns contain the data to be interpolated. Example:
</p>
<blockquote><pre>
table = [0,  0;
         1,  1;
         2,  4;
         4, 16]
If, e.g., the input u = 1.0, the output y =  1.0,
    e.g., the input u = 1.5, the output y =  2.5,
    e.g., the input u = 2.0, the output y =  4.0,
    e.g., the input u =-1.0, the output y = -1.0 (i.e., extrapolation).
</pre></blockquote>
<ul>
<li>The interpolation interval is found by a binary search where the interval used in the
    last call is used as start interval.</li>
<li>Via parameter <strong>smoothness</strong> it is defined how the data is interpolated:
<blockquote><pre>
smoothness = 1: Linear interpolation
           = 2: Akima interpolation: Smooth interpolation by cubic Hermite
                splines such that der(y) is continuous, also if extrapolated.
           = 3: Constant segments
           = 4: Fritsch-Butland interpolation: Smooth interpolation by cubic
                Hermite splines such that y preserves the monotonicity and
                der(y) is continuous, also if extrapolated.
           = 5: Steffen interpolation: Smooth interpolation by cubic Hermite
                splines such that y preserves the monotonicity and der(y)
                is continuous, also if extrapolated.
           = 6: Modified Akima interpolation: Smooth interpolation by cubic
                Hermite splines such that der(y) is continuous, also if
                extrapolated. Additionally, overshoots and edge cases of the
                original Akima interpolation method are avoided.
</pre></blockquote></li>
<li>First and second <strong>derivatives</strong> are provided, with exception of the following two smoothness options.
<ol>
<li>No derivatives are provided for interpolation by constant segments.</li>
<li>No second derivative is provided for linear interpolation.</li>
</ol></li>
<li>Values <strong>outside</strong> of the table range, are computed by
    extrapolation according to the setting of parameter <strong>extrapolation</strong>:
<blockquote><pre>
extrapolation = 1: Hold the first or last value of the table,
                   if outside of the table scope.
              = 2: Extrapolate by using the derivative at the first/last table
                   points if outside of the table scope.
                   (If smoothness is LinearSegments or ConstantSegments
                   this means to extrapolate linearly through the first/last
                   two table points.).
              = 3: Periodically repeat the table data (periodical function).
              = 4: No extrapolation, i.e. extrapolation triggers an error
</pre></blockquote></li>
<li>If the table has only <strong>one row</strong>, the table value is returned,
    independent of the value of the input signal.</li>
<li>The grid values (first column) have to be strictly increasing.</li>
</ul>
<p>
The table matrix can be defined in the following ways:
</p>
<ol>
<li>Explicitly supplied as <strong>parameter matrix</strong> \"table\",
    and the other parameters have the following values:
<blockquote><pre>
tableName is \"NoName\" or has only blanks,
fileName  is \"NoName\" or has only blanks.
</pre></blockquote></li>
<li><strong>Read</strong> from a <strong>file</strong> \"fileName\" where the matrix is stored as
    \"tableName\". Both text and MATLAB MAT-file format is possible.
    (The text format is described below).
    The MAT-file format comes in four different versions: v4, v6, v7 and v7.3.
    The library supports at least v4, v6 and v7 whereas v7.3 is optional.
    It is most convenient to generate the MAT-file from FreeMat or MATLAB&reg;
    by command
<blockquote><pre>
save tables.mat tab1 tab2 tab3
</pre></blockquote>
    or Scilab by command
<blockquote><pre>
savematfile tables.mat tab1 tab2 tab3
</pre></blockquote>
    when the three tables tab1, tab2, tab3 should be used from the model.<br>
    Note, a fileName can be defined as URI by using the helper function
    <a href=\"modelica://Modelica.Utilities.Files.loadResource\">loadResource</a>.</li>
<li>Statically stored in function \"usertab\" in file \"usertab.c\".
    The matrix is identified by \"tableName\". Parameter
    fileName = \"NoName\" or has only blanks. Row-wise storage is always to be
    preferred as otherwise the table is reallocated and transposed.
    See the <a href=\"modelica://Modelica.Blocks.Tables\">Tables</a> package
    documentation for more details.</li>
</ol>
<p>
When the constant \"NO_FILE_SYSTEM\" is defined, all file I/O related parts of the
source code are removed by the C-preprocessor, such that no access to files takes place.
</p>
<p>
If tables are read from a text file, the file needs to have the
following structure (\"-----\" is not part of the file content):
</p>
<blockquote><pre>
-----------------------------------------------------
#1
double tab1(5,2)   # comment line
  0   0
  1   1
  2   4
  3   9
  4  16
double tab2(5,2)   # another comment line
  0   0
  2   2
  4   8
  6  18
  8  32
-----------------------------------------------------
</pre></blockquote>
<p>
Note, that the first two characters in the file need to be
\"#1\" (a line comment defining the version number of the file format).
Afterwards, the corresponding matrix has to be declared
with type (= \"double\" or \"float\"), name and actual dimensions.
Finally, in successive rows of the file, the elements of the matrix
have to be given. The elements have to be provided as a sequence of
numbers in row-wise order (therefore a matrix row can span several
lines in the file and need not start at the beginning of a line).
Numbers have to be given according to C syntax (such as 2.3, -2, +2.e4).
Number separators are spaces, tab (\\t), comma (,), or semicolon (;).
Several matrices may be defined one after another. Line comments start
with the hash symbol (#) and can appear everywhere.
Text files should either be ASCII or UTF-8 encoded, where UTF-8 encoded strings are only allowed in line comments and an optional UTF-8 BOM at the start of the text file is ignored.
Other characters, like trailing non comments, are not allowed in the file.
</p>
<p>
MATLAB is a registered trademark of The MathWorks, Inc.
</p>
</html>"),
        Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Line(points={{-60.0,40.0},{-60.0,-40.0},{60.0,-40.0},{60.0,40.0},{30.0,40.0},{30.0,-40.0},{-30.0,-40.0},{-30.0,40.0},{-60.0,40.0},{-60.0,20.0},{60.0,20.0},{60.0,0.0},{-60.0,0.0},{-60.0,-20.0},{60.0,-20.0},{60.0,-40.0},{-60.0,-40.0},{-60.0,40.0},{60.0,40.0},{60.0,-40.0}}),
        Line(points={{0.0,40.0},{0.0,-40.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-60.0,20.0},{-30.0,40.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-60.0,0.0},{-30.0,20.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-60.0,-20.0},{-30.0,0.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-60.0,-40.0},{-30.0,-20.0}})}));
    end CombiTable1Ds;

    block CombiTable1Dv
      "Table look-up in one dimension (matrix/file) with n inputs and n outputs"
      extends Blocks.Interfaces.MIMOs(final n=size(columns, 1));
      parameter Boolean tableOnFile=false
        "= true, if table is defined on file or in function usertab"
        annotation (Dialog(group="Table data definition"));
      parameter Real table[:, :] = fill(0.0, 0, 2)
        "Table matrix (grid = first column; e.g., table=[0, 0; 1, 1; 2, 4])"
        annotation (Dialog(group="Table data definition",enable=not tableOnFile));
      parameter String tableName="NoName"
        "Table name on file or in function usertab (see docu)"
        annotation (Dialog(group="Table data definition",enable=tableOnFile));
      parameter String fileName="NoName" "File where matrix is stored"
        annotation (Dialog(
          group="Table data definition",
          enable=tableOnFile,
          loadSelector(filter="Text files (*.txt);;MATLAB MAT-files (*.mat)",
              caption="Open file in which table is present")));
      parameter Boolean verboseRead=true
        "= true, if info message that file is loading is to be printed"
        annotation (Dialog(group="Table data definition",enable=tableOnFile));
      parameter Integer columns[:]=2:size(table, 2)
        "Columns of table to be interpolated"
        annotation (Dialog(group="Table data interpretation"));
      parameter Blocks.Types.Smoothness smoothness=Blocks.Types.Smoothness.LinearSegments
        "Smoothness of table interpolation"
        annotation (Dialog(group="Table data interpretation"));
      parameter Blocks.Types.Extrapolation extrapolation=Blocks.Types.Extrapolation.LastTwoPoints
        "Extrapolation of data outside the definition range"
        annotation (Dialog(group="Table data interpretation"));
      parameter Boolean verboseExtrapolation=false
        "= true, if warning messages are to be printed if table input is outside the definition range"
        annotation (Dialog(group="Table data interpretation", enable=
              extrapolation == Blocks.Types.Extrapolation.LastTwoPoints or
              extrapolation == Blocks.Types.Extrapolation.HoldLastPoint));
      final parameter Real u_min=Internal.getTable1DAbscissaUmin(tableID)
        "Minimum abscissa value defined in table";
      final parameter Real u_max=Internal.getTable1DAbscissaUmax(tableID)
        "Maximum abscissa value defined in table";
    protected
      parameter Blocks.Types.ExternalCombiTable1D tableID=
          Blocks.Types.ExternalCombiTable1D(
              if tableOnFile then tableName else "NoName",
              if tableOnFile and fileName <> "NoName" and not
            Modelica.Utilities.Strings.isEmpty(fileName) then fileName else
            "NoName",
              table,
              columns,
              smoothness,
              extrapolation,
              if tableOnFile then verboseRead else false)
        "External table object";
    equation
      if tableOnFile then
        assert(tableName <> "NoName",
          "tableOnFile = true and no table name given");
      else
        assert(size(table, 1) > 0 and size(table, 2) > 0,
          "tableOnFile = false and parameter table is an empty matrix");
      end if;

      if verboseExtrapolation and (extrapolation == Blocks.Types.Extrapolation.LastTwoPoints
           or extrapolation == Blocks.Types.Extrapolation.HoldLastPoint) then
        for i in 1:n loop
          assert(noEvent(u[i] >= u_min), "
Extrapolation warning: The value u["   + String(i) +"] (=" + String(u[i]) + ") must be greater or equal
than the minimum abscissa value u_min (="   + String(u_min) + ") defined in the table.
",   level=AssertionLevel.warning);
          assert(noEvent(u[i] <= u_max), "
Extrapolation warning: The value u["   + String(i) +"] (=" + String(u[i]) + ") must be less or equal
than the maximum abscissa value u_max (="   + String(u_max) + ") defined in the table.
",   level=AssertionLevel.warning);
        end for;
      end if;

      if smoothness == Blocks.Types.Smoothness.ConstantSegments then
        for i in 1:n loop
          y[i] = Internal.getTable1DValueNoDer(tableID, i, u[i]);
        end for;
      elseif smoothness == Blocks.Types.Smoothness.LinearSegments then
        for i in 1:n loop
          y[i] = Internal.getTable1DValueNoDer2(tableID, i, u[i]);
        end for;
      else
        for i in 1:n loop
          y[i] = Internal.getTable1DValue(tableID, i, u[i]);
        end for;
      end if;
      annotation (
        Documentation(info="<html>
<p>
<strong>Univariate constant</strong>, <strong>linear</strong> or <strong>cubic Hermite
spline interpolation</strong> in <strong>one</strong> dimension of a
<strong>table</strong>.
Via parameter <strong>columns</strong> it can be defined how many columns of the
table are interpolated. If, e.g., columns={2,4}, it is assumed that 2 input
and 2 output signals are present and that the first output interpolates
the first input via column 2 and the second output interpolates the
second input via column 4 of the table matrix.
</p>
<p>
The grid points and function values are stored in a matrix \"table[i,j]\",
where the first column \"table[:,1]\" contains the grid points and the
other columns contain the data to be interpolated. Example:
</p>
<blockquote><pre>
table = [0,  0;
         1,  1;
         2,  4;
         4, 16]
If, e.g., the input u = 1.0, the output y =  1.0,
    e.g., the input u = 1.5, the output y =  2.5,
    e.g., the input u = 2.0, the output y =  4.0,
    e.g., the input u =-1.0, the output y = -1.0 (i.e., extrapolation).
</pre></blockquote>
<ul>
<li>The interpolation interval is found by a binary search where the interval used in the
    last call is used as start interval.</li>
<li>Via parameter <strong>smoothness</strong> it is defined how the data is interpolated:
<blockquote><pre>
smoothness = 1: Linear interpolation
           = 2: Akima interpolation: Smooth interpolation by cubic Hermite
                splines such that der(y) is continuous, also if extrapolated.
           = 3: Constant segments
           = 4: Fritsch-Butland interpolation: Smooth interpolation by cubic
                Hermite splines such that y preserves the monotonicity and
                der(y) is continuous, also if extrapolated.
           = 5: Steffen interpolation: Smooth interpolation by cubic Hermite
                splines such that y preserves the monotonicity and der(y)
                is continuous, also if extrapolated.
           = 6: Modified Akima interpolation: Smooth interpolation by cubic
                Hermite splines such that der(y) is continuous, also if
                extrapolated. Additionally, overshoots and edge cases of the
                original Akima interpolation method are avoided.
</pre></blockquote></li>
<li>First and second <strong>derivatives</strong> are provided, with exception of the following two smoothness options.
<ol>
<li>No derivatives are provided for interpolation by constant segments.</li>
<li>No second derivative is provided for linear interpolation.</li>
</ol></li>
<li>Values <strong>outside</strong> of the table range, are computed by
    extrapolation according to the setting of parameter <strong>extrapolation</strong>:
<blockquote><pre>
extrapolation = 1: Hold the first or last value of the table,
                   if outside of the table scope.
              = 2: Extrapolate by using the derivative at the first/last table
                   points if outside of the table scope.
                   (If smoothness is LinearSegments or ConstantSegments
                   this means to extrapolate linearly through the first/last
                   two table points.).
              = 3: Periodically repeat the table data (periodical function).
              = 4: No extrapolation, i.e. extrapolation triggers an error
</pre></blockquote></li>
<li>If the table has only <strong>one row</strong>, the table value is returned,
    independent of the value of the input signal.</li>
<li>The grid values (first column) have to be strictly increasing.</li>
</ul>
<p>
The table matrix can be defined in the following ways:
</p>
<ol>
<li>Explicitly supplied as <strong>parameter matrix</strong> \"table\",
    and the other parameters have the following values:
<blockquote><pre>
tableName is \"NoName\" or has only blanks,
fileName  is \"NoName\" or has only blanks.
</pre></blockquote></li>
<li><strong>Read</strong> from a <strong>file</strong> \"fileName\" where the matrix is stored as
    \"tableName\". Both text and MATLAB MAT-file format is possible.
    (The text format is described below).
    The MAT-file format comes in four different versions: v4, v6, v7 and v7.3.
    The library supports at least v4, v6 and v7 whereas v7.3 is optional.
    It is most convenient to generate the MAT-file from FreeMat or MATLAB&reg;
    by command
<blockquote><pre>
save tables.mat tab1 tab2 tab3
</pre></blockquote>
    or Scilab by command
<blockquote><pre>
savematfile tables.mat tab1 tab2 tab3
</pre></blockquote>
    when the three tables tab1, tab2, tab3 should be used from the model.<br>
    Note, a fileName can be defined as URI by using the helper function
    <a href=\"modelica://Modelica.Utilities.Files.loadResource\">loadResource</a>.</li>
<li>Statically stored in function \"usertab\" in file \"usertab.c\".
    The matrix is identified by \"tableName\". Parameter
    fileName = \"NoName\" or has only blanks. Row-wise storage is always to be
    preferred as otherwise the table is reallocated and transposed.
    See the <a href=\"modelica://Modelica.Blocks.Tables\">Tables</a> package
    documentation for more details.</li>
</ol>
<p>
When the constant \"NO_FILE_SYSTEM\" is defined, all file I/O related parts of the
source code are removed by the C-preprocessor, such that no access to files takes place.
</p>
<p>
If tables are read from a text file, the file needs to have the
following structure (\"-----\" is not part of the file content):
</p>
<blockquote><pre>
-----------------------------------------------------
#1
double tab1(5,2)   # comment line
  0   0
  1   1
  2   4
  3   9
  4  16
double tab2(5,2)   # another comment line
  0   0
  2   2
  4   8
  6  18
  8  32
-----------------------------------------------------
</pre></blockquote>
<p>
Note, that the first two characters in the file need to be
\"#1\" (a line comment defining the version number of the file format).
Afterwards, the corresponding matrix has to be declared
with type (= \"double\" or \"float\"), name and actual dimensions.
Finally, in successive rows of the file, the elements of the matrix
have to be given. The elements have to be provided as a sequence of
numbers in row-wise order (therefore a matrix row can span several
lines in the file and need not start at the beginning of a line).
Numbers have to be given according to C syntax (such as 2.3, -2, +2.e4).
Number separators are spaces, tab (\\t), comma (,), or semicolon (;).
Several matrices may be defined one after another. Line comments start
with the hash symbol (#) and can appear everywhere.
Text files should either be ASCII or UTF-8 encoded, where UTF-8 encoded strings are only allowed in line comments and an optional UTF-8 BOM at the start of the text file is ignored.
Other characters, like trailing non comments, are not allowed in the file.
</p>
<p>
MATLAB is a registered trademark of The MathWorks, Inc.
</p>
</html>"),
        Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Line(points={{-60.0,40.0},{-60.0,-40.0},{60.0,-40.0},{60.0,40.0},{30.0,40.0},{30.0,-40.0},{-30.0,-40.0},{-30.0,40.0},{-60.0,40.0},{-60.0,20.0},{60.0,20.0},{60.0,0.0},{-60.0,0.0},{-60.0,-20.0},{60.0,-20.0},{60.0,-40.0},{-60.0,-40.0},{-60.0,40.0},{60.0,40.0},{60.0,-40.0}}),
        Line(points={{0.0,40.0},{0.0,-40.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-60.0,20.0},{-30.0,40.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-60.0,0.0},{-30.0,20.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-60.0,-20.0},{-30.0,0.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-60.0,-40.0},{-30.0,-20.0}})}));
    end CombiTable1Dv;

    block CombiTable2Ds "Table look-up in two dimensions (matrix/file)"
      extends Blocks.Interfaces.SI2SO;
      extends Internal.CombiTable2DBase;
    equation
      if verboseExtrapolation and (extrapolation == Blocks.Types.Extrapolation.LastTwoPoints
           or extrapolation == Blocks.Types.Extrapolation.HoldLastPoint) then
        assert(noEvent(u1 >= u_min[1]), "
Extrapolation warning: The value u1 (="   + String(u1) + ") must be greater or equal
than the minimum abscissa value u_min[1] (="   + String(u_min[1]) + ") defined in the table.
",   level=AssertionLevel.warning);
        assert(noEvent(u1 <= u_max[1]), "
Extrapolation warning: The value u1 (="   + String(u1) + ") must be less or equal
than the maximum abscissa value u_max[1] (="   + String(u_max[1]) + ") defined in the table.
",   level=AssertionLevel.warning);
        assert(noEvent(u2 >= u_min[2]), "
Extrapolation warning: The value u2 (="   + String(u2) + ") must be greater or equal
than the minimum abscissa value u_min[2] (="   + String(u_min[2]) + ") defined in the table.
",   level=AssertionLevel.warning);
        assert(noEvent(u2 <= u_max[2]), "
Extrapolation warning: The value u2 (="   + String(u2) + ") must be less or equal
than the maximum abscissa value u_max[2] (="   + String(u_max[2]) + ") defined in the table.
",   level=AssertionLevel.warning);
      end if;

      if smoothness == Blocks.Types.Smoothness.ConstantSegments then
        y = Internal.getTable2DValueNoDer(tableID, u1, u2);
      elseif smoothness == Blocks.Types.Smoothness.LinearSegments then
        y = Internal.getTable2DValueNoDer2(tableID, u1, u2);
      else
        y = Internal.getTable2DValue(tableID, u1, u2);
      end if;
      annotation (
        Documentation(info="<html>
<p>
<strong>Bivariate constant</strong>, <strong>bilinear</strong> or <strong>bivariate
Akima interpolation</strong> of a <strong>two-dimensional table</strong>.
The grid points and function values are stored in a matrix \"table[i,j]\",
where:
</p>
<ul>
<li>the first column \"table[2:,1]\" contains the u1 grid points,</li>
<li>the first row \"table[1,2:]\" contains the u2 grid points,</li>
<li>the other rows and columns contain the data to be interpolated.</li>
</ul>
<p>
Example:
</p>
<blockquote><pre>
        |       |       |       |
        |  1.0  |  2.0  |  3.0  |  // u2
    ----*-------*-------*-------*
    1.0 |  1.0  |  3.0  |  5.0  |
    ----*-------*-------*-------*
    2.0 |  2.0  |  4.0  |  6.0  |
    ----*-------*-------*-------*
  // u1
is defined as
   table = [0.0,   1.0,   2.0,   3.0;
            1.0,   1.0,   3.0,   5.0;
            2.0,   2.0,   4.0,   6.0]
If, e.g., the input u1 is 1.0, input u2 is 1.0 and smoothness is LinearSegments, the output y is 1.0,
    e.g., the input u1 is 2.0, input u2 is 1.5 and smoothness is LinearSegments, the output y is 3.0.
</pre></blockquote>
<ul>
<li>The interpolation interval is found by a binary search where the interval used in the
    last call is used as start interval.</li>
<li>Via parameter <strong>smoothness</strong> it is defined how the data is interpolated:
<blockquote><pre>
smoothness = 1: Bilinear interpolation
           = 2: Bivariate Akima interpolation: Smooth interpolation by bicubic Hermite
                splines such that der(y) is continuous, also if extrapolated.
           = 3: Constant segments
           = 4: Fritsch-Butland interpolation: Not supported
           = 5: Steffen interpolation: Not supported
           = 6: Modified Akima interpolation: Not supported
</pre></blockquote></li>
<li>First and second <strong>derivatives</strong> are provided, with exception of the following two smoothness options.
<ol>
<li>No derivatives are provided for interpolation by constant segments.</li>
<li>No second derivative is provided for linear interpolation.</li>
</ol></li>
<li>Values <strong>outside</strong> of the table range, are computed by
    extrapolation according to the setting of parameter <strong>extrapolation</strong>:
<blockquote><pre>
extrapolation = 1: Hold the first or last values of the table,
                   if outside of the table scope.
              = 2: Extrapolate by using the derivative at the first/last table
                   points if outside of the table scope.
                   (If smoothness is LinearSegments or ConstantSegments
                   this means to extrapolate linearly through the first/last
                   two table points.).
              = 3: Periodically repeat the table data (periodical function).
              = 4: No extrapolation, i.e. extrapolation triggers an error
</pre></blockquote></li>
<li>If the table has only <strong>one element</strong>, the table value is returned,
    independent of the value of the input signal.</li>
<li>The grid values (first column and first row) have to be strictly
    increasing.</li>
</ul>
<p>
The table matrix can be defined in the following ways:
</p>
<ol>
<li>Explicitly supplied as <strong>parameter matrix</strong> \"table\",
    and the other parameters have the following values:
<blockquote><pre>
tableName is \"NoName\" or has only blanks,
fileName  is \"NoName\" or has only blanks.
</pre></blockquote></li>
<li><strong>Read</strong> from a <strong>file</strong> \"fileName\" where the matrix is stored as
    \"tableName\". Both text and MATLAB MAT-file format is possible.
    (The text format is described below).
    The MAT-file format comes in four different versions: v4, v6, v7 and v7.3.
    The library supports at least v4, v6 and v7 whereas v7.3 is optional.
    It is most convenient to generate the MAT-file from FreeMat or MATLAB&reg;
    by command
<blockquote><pre>
save tables.mat tab1 tab2 tab3
</pre></blockquote>
    or Scilab by command
<blockquote><pre>
savematfile tables.mat tab1 tab2 tab3
</pre></blockquote>
    when the three tables tab1, tab2, tab3 should be used from the model.<br>
    Note, a fileName can be defined as URI by using the helper function
    <a href=\"modelica://Modelica.Utilities.Files.loadResource\">loadResource</a>.</li>
<li>Statically stored in function \"usertab\" in file \"usertab.c\".
    The matrix is identified by \"tableName\". Parameter
    fileName = \"NoName\" or has only blanks. Row-wise storage is always to be
    preferred as otherwise the table is reallocated and transposed.
    See the <a href=\"modelica://Modelica.Blocks.Tables\">Tables</a> package
    documentation for more details.</li>
</ol>
<p>
When the constant \"NO_FILE_SYSTEM\" is defined, all file I/O related parts of the
source code are removed by the C-preprocessor, such that no access to files takes place.
</p>
<p>
If tables are read from a text file, the file needs to have the
following structure (\"-----\" is not part of the file content):
</p>
<blockquote><pre>
-----------------------------------------------------
#1
double table2D_1(3,4)   # comment line
0.0  1.0  2.0  3.0  # u[2] grid points
1.0  1.0  3.0  5.0
2.0  2.0  4.0  6.0

double table2D_2(4,4)   # comment line
0.0  1.0  2.0  3.0  # u[2] grid points
1.0  1.0  3.0  5.0
2.0  2.0  4.0  6.0
3.0  3.0  5.0  7.0
-----------------------------------------------------
</pre></blockquote>
<p>
Note, that the first two characters in the file need to be
\"#1\" (a line comment defining the version number of the file format).
Afterwards, the corresponding matrix has to be declared
with type (= \"double\" or \"float\"), name and actual dimensions.
Finally, in successive rows of the file, the elements of the matrix
have to be given. The elements have to be provided as a sequence of
numbers in row-wise order (therefore a matrix row can span several
lines in the file and need not start at the beginning of a line).
Numbers have to be given according to C syntax (such as 2.3, -2, +2.e4).
Number separators are spaces, tab (\\t), comma (,), or semicolon (;).
Several matrices may be defined one after another. Line comments start
with the hash symbol (#) and can appear everywhere.
Text files should either be ASCII or UTF-8 encoded, where UTF-8 encoded strings are only allowed in line comments and an optional UTF-8 BOM at the start of the text file is ignored.
Other characters, like trailing non comments, are not allowed in the file.
The matrix elements are interpreted in exactly the same way
as if the matrix is given as a parameter. For example, the first
column \"table2D_1[2:,1]\" contains the u[1] grid points,
and the first row \"table2D_1[1,2:]\" contains the u[2] grid points.
</p>
<p>
MATLAB is a registered trademark of The MathWorks, Inc.
</p>
</html>"));
    end CombiTable2Ds;

    block CombiTable2Dv "Table look-up in two dimensions (matrix/file) with vector inputs and vector output of size n"
      extends Blocks.Interfaces.MI2MO;
      extends Internal.CombiTable2DBase;
    equation
      if verboseExtrapolation and (extrapolation == Blocks.Types.Extrapolation.LastTwoPoints
           or extrapolation == Blocks.Types.Extrapolation.HoldLastPoint) then
        for j in 1:n loop
          assert(noEvent(u1[j] >= u_min[1]), "
Extrapolation warning: The value u1["   + String(j) + "] (=" + String(u1[j]) + ") must be greater or equal
than the minimum abscissa value u_min[1] (="   + String(u_min[1]) + ") defined in the table.
",   level=AssertionLevel.warning);
          assert(noEvent(u1[j] <= u_max[1]), "
Extrapolation warning: The value u1["   + String(j) + "] (=" + String(u1[j]) + ") must be less or equal
than the maximum abscissa value u_max[1] (="   + String(u_max[1]) + ") defined in the table.
",   level=AssertionLevel.warning);
          assert(noEvent(u2[j] >= u_min[2]), "
Extrapolation warning: The value u2["   + String(j) + "] (=" + String(u2[j]) + ") must be greater or equal
than the minimum abscissa value u_min[2] (="   + String(u_min[2]) + ") defined in the table.
",   level=AssertionLevel.warning);
          assert(noEvent(u2[j] <= u_max[2]), "
Extrapolation warning: The value u2["   + String(j) + "] (=" + String(u2[j]) + ") must be less or equal
than the maximum abscissa value u_max[2] (="   + String(u_max[2]) + ") defined in the table.
",   level=AssertionLevel.warning);
        end for;
      end if;

      if smoothness == Blocks.Types.Smoothness.ConstantSegments then
        for j in 1:n loop
          y[j] = Blocks.Tables.Internal.getTable2DValueNoDer(
                tableID,
                u1[j],
                u2[j]);
        end for;
      elseif smoothness == Blocks.Types.Smoothness.LinearSegments then
        for j in 1:n loop
          y[j] = Blocks.Tables.Internal.getTable2DValueNoDer2(
                tableID,
                u1[j],
                u2[j]);
        end for;
      else
        for j in 1:n loop
          y[j] = Blocks.Tables.Internal.getTable2DValue(
                tableID,
                u1[j],
                u2[j]);
        end for;
      end if;
    annotation(Documentation(info="<html>
<p>
<strong>Bivariate constant</strong>, <strong>bilinear</strong> or <strong>bivariate
Akima interpolation</strong> of a <strong>two-dimensional table</strong>.
The grid points and function values are stored in a matrix \"table[i,j]\",
where:
</p>
<ul>
<li>the first column \"table[2:,1]\" contains the u1 grid points,</li>
<li>the first row \"table[1,2:]\" contains the u2 grid points,</li>
<li>the other rows and columns contain the data to be interpolated.</li>
</ul>
<p>
Example:
</p>
<blockquote><pre>
        |       |       |       |
        |  1.0  |  2.0  |  3.0  |  // u2
    ----*-------*-------*-------*
    1.0 |  1.0  |  3.0  |  5.0  |
    ----*-------*-------*-------*
    2.0 |  2.0  |  4.0  |  6.0  |
    ----*-------*-------*-------*
  // u1
is defined as
   table = [0.0,   1.0,   2.0,   3.0;
            1.0,   1.0,   3.0,   5.0;
            2.0,   2.0,   4.0,   6.0]
If, e.g., the input u1 is {1.0}, input u2 is {1.0} and smoothness is LinearSegments, the output y is {1.0},
    e.g., the input u1 is {2.0}, input u2 is {1.5} and smoothness is LinearSegments, the output y is {3.0}.
</pre></blockquote>
<ul>
<li>The interpolation interval is found by a binary search where the interval used in the
    last call is used as start interval.</li>
<li>Via parameter <strong>smoothness</strong> it is defined how the data is interpolated:
<blockquote><pre>
smoothness = 1: Bilinear interpolation
           = 2: Bivariate Akima interpolation: Smooth interpolation by bicubic Hermite
                splines such that der(y) is continuous, also if extrapolated.
           = 3: Constant segments
           = 4: Fritsch-Butland interpolation: Not supported
           = 5: Steffen interpolation: Not supported
           = 6: Modified Akima interpolation: Not supported
</pre></blockquote></li>
<li>First and second <strong>derivatives</strong> are provided, with exception of the following two smoothness options.
<ol>
<li>No derivatives are provided for interpolation by constant segments.</li>
<li>No second derivative is provided for linear interpolation.</li>
</ol></li>
<li>Values <strong>outside</strong> of the table range, are computed by
    extrapolation according to the setting of parameter <strong>extrapolation</strong>:
<blockquote><pre>
extrapolation = 1: Hold the first or last values of the table,
                   if outside of the table scope.
              = 2: Extrapolate by using the derivative at the first/last table
                   points if outside of the table scope.
                   (If smoothness is LinearSegments or ConstantSegments
                   this means to extrapolate linearly through the first/last
                   two table points.).
              = 3: Periodically repeat the table data (periodical function).
              = 4: No extrapolation, i.e. extrapolation triggers an error
</pre></blockquote></li>
<li>If the table has only <strong>one element</strong>, the table value is returned,
    independent of the value of the input signal.</li>
<li>The grid values (first column and first row) have to be strictly
    increasing.</li>
</ul>
<p>
The table matrix can be defined in the following ways:
</p>
<ol>
<li>Explicitly supplied as <strong>parameter matrix</strong> \"table\",
    and the other parameters have the following values:
<blockquote><pre>
tableName is \"NoName\" or has only blanks,
fileName  is \"NoName\" or has only blanks.
</pre></blockquote></li>
<li><strong>Read</strong> from a <strong>file</strong> \"fileName\" where the matrix is stored as
    \"tableName\". Both text and MATLAB MAT-file format is possible.
    (The text format is described below).
    The MAT-file format comes in four different versions: v4, v6, v7 and v7.3.
    The library supports at least v4, v6 and v7 whereas v7.3 is optional.
    It is most convenient to generate the MAT-file from FreeMat or MATLAB&reg;
    by command
<blockquote><pre>
save tables.mat tab1 tab2 tab3
</pre></blockquote>
    or Scilab by command
<blockquote><pre>
savematfile tables.mat tab1 tab2 tab3
</pre></blockquote>
    when the three tables tab1, tab2, tab3 should be used from the model.<br>
    Note, a fileName can be defined as URI by using the helper function
    <a href=\"modelica://Modelica.Utilities.Files.loadResource\">loadResource</a>.</li>
<li>Statically stored in function \"usertab\" in file \"usertab.c\".
    The matrix is identified by \"tableName\". Parameter
    fileName = \"NoName\" or has only blanks. Row-wise storage is always to be
    preferred as otherwise the table is reallocated and transposed.
    See the <a href=\"modelica://Modelica.Blocks.Tables\">Tables</a> package
    documentation for more details.</li>
</ol>
<p>
When the constant \"NO_FILE_SYSTEM\" is defined, all file I/O related parts of the
source code are removed by the C-preprocessor, such that no access to files takes place.
</p>
<p>
If tables are read from a text file, the file needs to have the
following structure (\"-----\" is not part of the file content):
</p>
<blockquote><pre>
-----------------------------------------------------
#1
double table2D_1(3,4)   # comment line
0.0  1.0  2.0  3.0  # u[2] grid points
1.0  1.0  3.0  5.0
2.0  2.0  4.0  6.0

double table2D_2(4,4)   # comment line
0.0  1.0  2.0  3.0  # u[2] grid points
1.0  1.0  3.0  5.0
2.0  2.0  4.0  6.0
3.0  3.0  5.0  7.0
-----------------------------------------------------
</pre></blockquote>
<p>
Note, that the first two characters in the file need to be
\"#1\" (a line comment defining the version number of the file format).
Afterwards, the corresponding matrix has to be declared
with type (= \"double\" or \"float\"), name and actual dimensions.
Finally, in successive rows of the file, the elements of the matrix
have to be given. The elements have to be provided as a sequence of
numbers in row-wise order (therefore a matrix row can span several
lines in the file and need not start at the beginning of a line).
Numbers have to be given according to C syntax (such as 2.3, -2, +2.e4).
Number separators are spaces, tab (\\t), comma (,), or semicolon (;).
Several matrices may be defined one after another. Line comments start
with the hash symbol (#) and can appear everywhere.
Text files should either be ASCII or UTF-8 encoded, where UTF-8 encoded strings are only allowed in line comments and an optional UTF-8 BOM at the start of the text file is ignored.
Other characters, like trailing non comments, are not allowed in the file.
The matrix elements are interpreted in exactly the same way
as if the matrix is given as a parameter. For example, the first
column \"table2D_1[2:,1]\" contains the u[1] grid points,
and the first row \"table2D_1[1,2:]\" contains the u[2] grid points.
</p>
<p>
MATLAB is a registered trademark of The MathWorks, Inc.
</p>
</html>"));
    end CombiTable2Dv;

    package Internal "Internal external object definitions for table functions that should not be directly utilized by the user"
      extends Modelica.Icons.InternalPackage;
      partial block CombiTable2DBase "Base class for variants of table look-up in two dimensions"
        parameter Boolean tableOnFile=false
          "= true, if table is defined on file or in function usertab"
          annotation (Dialog(group="Table data definition"));
        parameter Real table[:, :] = fill(0.0, 0, 2)
          "Table matrix (grid u1 = first column, grid u2 = first row; e.g., table=[0, 0; 0, 1])"
          annotation (Dialog(group="Table data definition",enable=not tableOnFile));
        parameter String tableName="NoName"
          "Table name on file or in function usertab (see docu)"
          annotation (Dialog(group="Table data definition",enable=tableOnFile));
        parameter String fileName="NoName" "File where matrix is stored"
          annotation (Dialog(
            group="Table data definition",
            enable=tableOnFile,
            loadSelector(filter="Text files (*.txt);;MATLAB MAT-files (*.mat)",
                caption="Open file in which table is present")));
        parameter Boolean verboseRead=true
          "= true, if info message that file is loading is to be printed"
          annotation (Dialog(group="Table data definition",enable=tableOnFile));
        parameter Blocks.Types.Smoothness smoothness=Blocks.Types.Smoothness.LinearSegments
          "Smoothness of table interpolation"
          annotation (Dialog(group="Table data interpretation"));
        parameter Blocks.Types.Extrapolation extrapolation=Blocks.Types.Extrapolation.LastTwoPoints
          "Extrapolation of data outside the definition range"
          annotation (Dialog(group="Table data interpretation"));
        parameter Boolean verboseExtrapolation=false
          "= true, if warning messages are to be printed if table input is outside the definition range"
          annotation (Dialog(group="Table data interpretation", enable=
                extrapolation == Blocks.Types.Extrapolation.LastTwoPoints or
                extrapolation == Blocks.Types.Extrapolation.HoldLastPoint));
        final parameter Real u_min[2]=getTable2DAbscissaUmin(tableID)
          "Minimum abscissa value defined in table";
        final parameter Real u_max[2]=getTable2DAbscissaUmax(tableID)
          "Maximum abscissa value defined in table";
      protected
          parameter Blocks.Types.ExternalCombiTable2D tableID=
            Blocks.Types.ExternalCombiTable2D(
                  if tableOnFile then tableName else "NoName",
                  if tableOnFile and fileName <> "NoName" and not
              Modelica.Utilities.Strings.isEmpty(fileName) then fileName else
              "NoName",
                  table,
                  smoothness,
                  extrapolation,
                  if tableOnFile then verboseRead else false)
          "External table object";
      equation
          if tableOnFile then
            assert(tableName <> "NoName",
              "tableOnFile = true and no table name given");
          else
            assert(size(table, 1) > 0 and size(table, 2) > 0,
              "tableOnFile = false and parameter table is an empty matrix");
          end if;
        annotation(Icon(
        coordinateSystem(preserveAspectRatio=true,
          extent={{-100.0,-100.0},{100.0,100.0}}),
          graphics={
        Line(points={{-60.0,40.0},{-60.0,-40.0},{60.0,-40.0},{60.0,40.0},{30.0,40.0},{30.0,-40.0},{-30.0,-40.0},{-30.0,40.0},{-60.0,40.0},{-60.0,20.0},{60.0,20.0},{60.0,0.0},{-60.0,0.0},{-60.0,-20.0},{60.0,-20.0},{60.0,-40.0},{-60.0,-40.0},{-60.0,40.0},{60.0,40.0},{60.0,-40.0}}),
        Line(points={{0.0,40.0},{0.0,-40.0}}),
        Line(points={{-60.0,40.0},{-30.0,20.0}}),
        Line(points={{-30.0,40.0},{-60.0,20.0}}),
        Rectangle(origin={2.3077,-0.0},
          fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-62.3077,0.0},{-32.3077,20.0}}),
        Rectangle(origin={2.3077,-0.0},
          fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-62.3077,-20.0},{-32.3077,0.0}}),
        Rectangle(origin={2.3077,-0.0},
          fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-62.3077,-40.0},{-32.3077,-20.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{-30.0,20.0},{0.0,40.0}}),
        Rectangle(fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{0.0,20.0},{30.0,40.0}}),
        Rectangle(origin={-2.3077,-0.0},
          fillColor={255,215,136},
          fillPattern=FillPattern.Solid,
          extent={{32.3077,20.0},{62.3077,40.0}})}));
      end CombiTable2DBase;

      pure function getTimeTableValue
        "Interpolate 1-dim. table where first column is time"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        input Integer icol "Column number";
        input Real timeIn "(Scaled) time value";
        discrete input Real nextTimeEvent "(Scaled) next time event in table";
        discrete input Real pre_nextTimeEvent "Pre-value of (scaled) next time event in table";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTimeTable_getValue(tableID, icol, timeIn, nextTimeEvent, pre_nextTimeEvent)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative(
            noDerivative=nextTimeEvent,
            noDerivative=pre_nextTimeEvent) = getDerTimeTableValue);
      end getTimeTableValue;

      pure function getTimeTableValueNoDer
        "Interpolate 1-dim. table where first column is time (but do not provide a derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        input Integer icol "Column number";
        input Real timeIn "(Scaled) time value";
        discrete input Real nextTimeEvent "(Scaled) next time event in table";
        discrete input Real pre_nextTimeEvent "Pre-value of (scaled) next time event in table";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTimeTable_getValue(tableID, icol, timeIn, nextTimeEvent, pre_nextTimeEvent)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTimeTableValueNoDer;

      pure function getTimeTableValueNoDer2
        "Interpolate 1-dim. table where first column is time (but do not provide a second derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        input Integer icol "Column number";
        input Real timeIn "(Scaled) time value";
        discrete input Real nextTimeEvent "(Scaled) next time event in table";
        discrete input Real pre_nextTimeEvent "Pre-value of (scaled) next time event in table";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTimeTable_getValue(tableID, icol, timeIn, nextTimeEvent, pre_nextTimeEvent)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative(
            noDerivative=nextTimeEvent,
            noDerivative=pre_nextTimeEvent) = getDerTimeTableValueNoDer);
      end getTimeTableValueNoDer2;

      pure function getDerTimeTableValue
        "Derivative of interpolated 1-dim. table where first column is time"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        input Integer icol "Column number";
        input Real timeIn "(Scaled) time value";
        discrete input Real nextTimeEvent "(Scaled) next time event in table";
        discrete input Real pre_nextTimeEvent "Pre-value of (scaled) next time event in table";
        input Real der_timeIn "Derivative of (scaled) time value";
        output Real der_y "Derivative of interpolated value";
        external "C" der_y = ModelicaStandardTables_CombiTimeTable_getDerValue(tableID, icol, timeIn, nextTimeEvent, pre_nextTimeEvent, der_timeIn)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative(
            order=2,
            noDerivative=nextTimeEvent,
            noDerivative=pre_nextTimeEvent) = getDer2TimeTableValue);
      end getDerTimeTableValue;

      pure function getDerTimeTableValueNoDer
        "Derivative of interpolated 1-dim. table where first column is time (but do not provide a derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        input Integer icol "Column number";
        input Real timeIn "(Scaled) time value";
        discrete input Real nextTimeEvent "(Scaled) next time event in table";
        discrete input Real pre_nextTimeEvent "Pre-value of (scaled) next time event in table";
        input Real der_timeIn "Derivative of (scaled) time value";
        output Real der_y "Derivative of interpolated value";
        external "C" der_y = ModelicaStandardTables_CombiTimeTable_getDerValue(tableID, icol, timeIn, nextTimeEvent, pre_nextTimeEvent, der_timeIn)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getDerTimeTableValueNoDer;

      pure function getDer2TimeTableValue
        "Second derivative of interpolated 1-dim. table where first column is time"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        input Integer icol "Column number";
        input Real timeIn "(Scaled) time value";
        discrete input Real nextTimeEvent "(Scaled) next time event in table";
        discrete input Real pre_nextTimeEvent "Pre-value of (scaled) next time event in table";
        input Real der_timeIn "Derivative of (scaled) time value";
        input Real der2_timeIn "Second derivative of (scaled) time value";
        output Real der2_y "Second derivative of interpolated value";
        external "C" der2_y = ModelicaStandardTables_CombiTimeTable_getDer2Value(tableID, icol, timeIn, nextTimeEvent, pre_nextTimeEvent, der_timeIn, der2_timeIn)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getDer2TimeTableValue;

      pure function getTimeTableTmin
        "Return minimum abscissa value of 1-dim. table where first column is time"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        output Real timeMin "Minimum abscissa value in table";
        external "C" timeMin = ModelicaStandardTables_CombiTimeTable_minimumTime(tableID)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTimeTableTmin;

      pure function getTimeTableTmax
        "Return maximum abscissa value of 1-dim. table where first column is time"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        output Real timeMax "Maximum abscissa value in table";
        external "C" timeMax = ModelicaStandardTables_CombiTimeTable_maximumTime(tableID)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTimeTableTmax;

      pure function getNextTimeEvent
        "Return next time event value of 1-dim. table where first column is time"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTimeTable tableID
          "External table object";
        input Real timeIn "(Scaled) time value";
        output Real nextTimeEvent "(Scaled) next time event in table";
        external "C" nextTimeEvent = ModelicaStandardTables_CombiTimeTable_nextTimeEvent(tableID, timeIn)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getNextTimeEvent;

      pure function getTable1DValue "Interpolate 1-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable1D tableID "External table object";
        input Integer icol "Column number";
        input Real u "Abscissa value";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTable1D_getValue(tableID, icol, u)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative = getDerTable1DValue);
      end getTable1DValue;

      pure function getTable1DValueNoDer
        "Interpolate 1-dim. table defined by matrix (but do not provide a derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable1D tableID "External table object";
        input Integer icol "Column number";
        input Real u "Abscissa value";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTable1D_getValue(tableID, icol, u)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTable1DValueNoDer;

      pure function getTable1DValueNoDer2
        "Interpolate 1-dim. table defined by matrix (but do not provide a second derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable1D tableID "External table object";
        input Integer icol "Column number";
        input Real u "Abscissa value";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTable1D_getValue(tableID, icol, u)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative = getDerTable1DValueNoDer);
      end getTable1DValueNoDer2;

      pure function getDerTable1DValue
        "Derivative of interpolated 1-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable1D tableID "External table object";
        input Integer icol "Column number";
        input Real u "Abscissa value";
        input Real der_u "Derivative of abscissa value";
        output Real der_y "Derivative of interpolated value";
        external "C" der_y = ModelicaStandardTables_CombiTable1D_getDerValue(tableID, icol, u, der_u)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative(order=2) = getDer2Table1DValue);
      end getDerTable1DValue;

      pure function getDerTable1DValueNoDer
        "Derivative of interpolated 1-dim. table defined by matrix (but do not provide a second derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable1D tableID "External table object";
        input Integer icol "Column number";
        input Real u "Abscissa value";
        input Real der_u "Derivative of abscissa value";
        output Real der_y "Derivative of interpolated value";
        external "C" der_y = ModelicaStandardTables_CombiTable1D_getDerValue(tableID, icol, u, der_u)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getDerTable1DValueNoDer;

      pure function getDer2Table1DValue
        "Second derivative of interpolated 1-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable1D tableID "External table object";
        input Integer icol "Column number";
        input Real u "Abscissa value";
        input Real der_u "Derivative of abscissa value";
        input Real der2_u " Second derivative of abscissa value";
        output Real der2_y "Second derivative of interpolated value";
        external "C" der2_y = ModelicaStandardTables_CombiTable1D_getDer2Value(tableID, icol, u, der_u, der2_u)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getDer2Table1DValue;

      pure function getTable1DAbscissaUmin
        "Return minimum abscissa value of 1-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable1D tableID "External table object";
        output Real uMin "Minimum abscissa value in table";
        external "C" uMin = ModelicaStandardTables_CombiTable1D_minimumAbscissa(tableID)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTable1DAbscissaUmin;

      pure function getTable1DAbscissaUmax
        "Return maximum abscissa value of 1-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable1D tableID "External table object";
        output Real uMax "Maximum abscissa value in table";
        external "C" uMax = ModelicaStandardTables_CombiTable1D_maximumAbscissa(tableID)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTable1DAbscissaUmax;

      pure function getTable2DValue "Interpolate 2-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable2D tableID "External table object";
        input Real u1 "Value of first independent variable";
        input Real u2 "Value of second independent variable";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTable2D_getValue(tableID, u1, u2)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative = getDerTable2DValue);
      end getTable2DValue;

      pure function getTable2DValueNoDer
        "Interpolate 2-dim. table defined by matrix (but do not provide a derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable2D tableID "External table object";
        input Real u1 "Value of first independent variable";
        input Real u2 "Value of second independent variable";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTable2D_getValue(tableID, u1, u2)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTable2DValueNoDer;

      pure function getTable2DValueNoDer2
        "Interpolate 2-dim. table defined by matrix (but do not provide a second derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable2D tableID "External table object";
        input Real u1 "Value of first independent variable";
        input Real u2 "Value of second independent variable";
        output Real y "Interpolated value";
        external "C" y = ModelicaStandardTables_CombiTable2D_getValue(tableID, u1, u2)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative = getDerTable2DValueNoDer);
      end getTable2DValueNoDer2;

      pure function getDerTable2DValue
        "Derivative of interpolated 2-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable2D tableID "External table object";
        input Real u1 "Value of first independent variable";
        input Real u2 "Value of second independent variable";
        input Real der_u1 "Derivative of first independent variable";
        input Real der_u2 "Derivative of second independent variable";
        output Real der_y "Derivative of interpolated value";
        external "C" der_y = ModelicaStandardTables_CombiTable2D_getDerValue(tableID, u1, u2, der_u1, der_u2)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
        annotation (derivative(order=2) = getDer2Table2DValue);
      end getDerTable2DValue;

      pure function getDerTable2DValueNoDer
        "Derivative of interpolated 2-dim. table defined by matrix (but do not provide a second derivative function)"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable2D tableID "External table object";
        input Real u1 "Value of first independent variable";
        input Real u2 "Value of second independent variable";
        input Real der_u1 "Derivative of first independent variable";
        input Real der_u2 "Derivative of second independent variable";
        output Real der_y "Derivative of interpolated value";
        external "C" der_y = ModelicaStandardTables_CombiTable2D_getDerValue(tableID, u1, u2, der_u1, der_u2)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getDerTable2DValueNoDer;

      pure function getDer2Table2DValue
        "Second derivative of interpolated 2-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable2D tableID "External table object";
        input Real u1 "Value of first independent variable";
        input Real u2 "Value of second independent variable";
        input Real der_u1 "Derivative of first independent variable";
        input Real der_u2 "Derivative of second independent variable";
        input Real der2_u1 "Second derivative of first independent variable";
        input Real der2_u2 "Second derivative of second independent variable";
        output Real der2_y "Second derivative of interpolated value";
        external "C" der2_y = ModelicaStandardTables_CombiTable2D_getDer2Value(tableID, u1, u2, der_u1, der_u2, der2_u1, der2_u2)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getDer2Table2DValue;

      pure function getTable2DAbscissaUmin
        "Return minimum abscissa value of 2-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable2D tableID "External table object";
        output Real uMin[2] "Minimum abscissa value in table";
        external "C" ModelicaStandardTables_CombiTable2D_minimumAbscissa(tableID, uMin)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTable2DAbscissaUmin;

      pure function getTable2DAbscissaUmax
        "Return maximum abscissa value of 2-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input Blocks.Types.ExternalCombiTable2D tableID "External table object";
        output Real uMax[2] "Maximum abscissa value in table";
        external "C" ModelicaStandardTables_CombiTable2D_maximumAbscissa(tableID, uMax)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end getTable2DAbscissaUmax;
    end Internal;
    annotation (Documentation(info="<html>
<p>This package contains blocks for one- and two-dimensional interpolation in tables.</p>
<h4>Special interest topic: Statically stored tables for real-time simulation targets</h4>
<p>Especially for use on real-time platform targets (e.g., HIL-simulators) with <strong>no file system</strong>, it is possible to statically
store tables using a function &quot;usertab&quot; in a file conventionally named &quot;usertab.c&quot;. This can be more efficient than providing the tables as Modelica parameter arrays.</p>
<p>This is achieved by providing the tables in a specific structure as C-code and compiling that C-code together with the rest of the simulation model into a binary
that can be executed on the target platform. The &quot;Resources/Data/Tables/&quot; subdirectory of the MSL installation directory contains the files
<a href=\"modelica://Modelica/Resources/Data/Tables/usertab.c\">&quot;usertab.c&quot;</a> and <a href=\"modelica://Modelica/Resources/Data/Tables/usertab.h\">&quot;usertab.h&quot;</a>
that can be used as a template for own developments. While &quot;usertab.c&quot; would be typically used unmodified, the
&quot;usertab.h&quot; needs to adapted for the own needs.</p>
<p>In order to work it is necessary that the compiler pulls in the &quot;usertab.c&quot; file. Different Modelica tools might provide different mechanisms to do so.
Please consult the respective documentation/support for your Modelica tool.</p>
<p>A possible (though slightly makeshift) approach is to pull in the required files by utilizing a &quot;dummy&quot;-function that uses the Modelica external function
interface to include the required &quot;usertab.c&quot;. An example how this can be done is given below.</p>
<blockquote><pre>
model ExampleCTable \"Example utilizing the usertab.c interface\"
  extends Modelica.Icons.Example;
  parameter Real dummy(fixed=false) \"Dummy parameter\" annotation(HideResult=true);
  Modelica.Blocks.Tables.CombiTable1Dv table(tableOnFile=true, tableName=\"TestTable_1D_a\")
    annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
  Modelica.Blocks.Sources.ContinuousClock clock
    annotation (Placement(transformation(extent={{-80,0},{-60,20}})));
protected
  encapsulated impure function getUsertab \"External dummy function to include \\\"usertab.c\\\"\"
    input Real dummy_u[:];
    output Real dummy_y;
    external \"C\" dummy_y = mydummyfunc(dummy_u);
    annotation(IncludeDirectory=\"modelica://Modelica/Resources/Data/Tables\",
           Include = \"#include \"usertab.c\"
double mydummyfunc(double* dummy_in) {
   return 0;
}
\");
  end getUsertab;
initial equation
  dummy = getUsertab(table.y);
equation
  connect(clock.y, table.u[1]) annotation (Line(points={{-59,10},{-42,10}}, color={0,0,127}));
  annotation (experiment(StartTime=0, StopTime=5), uses(Modelica(version=\"4.0.0\")));
end ExampleCTable;
</pre></blockquote>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
              {100,100}}), graphics={
          Rectangle(
            extent={{-76,-26},{80,-76}},
            lineColor={95,95,95},
            fillColor={235,235,235},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-76,24},{80,-26}},
            lineColor={95,95,95},
            fillColor={235,235,235},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-76,74},{80,24}},
            lineColor={95,95,95},
            fillColor={235,235,235},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-28,74},{-28,-76}},
            color={95,95,95}),
          Line(
            points={{24,74},{24,-76}},
            color={95,95,95})}));
  end Tables;

  package Types
    "Library of constants, external objects and types with choices, especially to build menus"
    extends Modelica.Icons.TypesPackage;

    type Smoothness = enumeration(
        LinearSegments "Linear interpolation of table points",
        ContinuousDerivative
          "Akima spline interpolation of table points (such that the first derivative is continuous)",
        ConstantSegments
          "Piecewise constant interpolation of table points (the value from the previous abscissa point is returned)",
        MonotoneContinuousDerivative1
          "Fritsch-Butland spline interpolation (such that the monotonicity is preserved and the first derivative is continuous)",
        MonotoneContinuousDerivative2
          "Steffen spline interpolation of table points (such that the monotonicity is preserved and the first derivative is continuous)",
        ModifiedContinuousDerivative
          "Modified Akima spline interpolation of table points (such that the first derivative is continuous and shortcomings of the original Akima method are avoided)")
      "Enumeration defining the smoothness of table interpolation";

      type Extrapolation = enumeration(
        HoldLastPoint
          "Hold the first/last table point outside of the table scope",
        LastTwoPoints
          "Extrapolate by using the derivative at the first/last table points outside of the table scope",
        Periodic "Repeat the table scope periodically",
        NoExtrapolation "Extrapolation triggers an error")
      "Enumeration defining the extrapolation of table interpolation";

      type TimeEvents = enumeration(
        Always "Always generate time events at interval boundaries",
        AtDiscontinuities "Generate time events at discontinuities (defined by duplicated sample points)",
        NoTimeEvents "No time events at interval boundaries")
      "Enumeration defining the time event handling of time table interpolation";

      type Init = enumeration(
        NoInit
          "No initialization (start values are used as guess values with fixed=false)",
        SteadyState
          "Steady state initialization (derivatives of states are zero)",
        InitialState "Initialization with initial states",
        InitialOutput
          "Initialization with initial outputs (and steady state of the states if possible)")
      "Enumeration defining initialization of a block" annotation (Evaluate=true,
      Documentation(info="<html>
  <p>The following initialization alternatives are available:</p>
  <dl>
    <dt><code><strong>NoInit</strong></code></dt>
      <dd>No initialization (start values are used as guess values with <code>fixed=false</code>)</dd>
    <dt><code><strong>SteadyState</strong></code></dt>
      <dd>Steady state initialization (derivatives of states are zero)</dd>
    <dt><code><strong>InitialState</strong></code></dt>
      <dd>Initialization with initial states</dd>
    <dt><code><strong>InitialOutput</strong></code></dt>
      <dd>Initialization with initial outputs (and steady state of the states if possible)</dd>
  </dl>
</html>"));

     type SimpleController = enumeration(
        P "P controller",
        PI "PI controller",
        PD "PD controller",
        PID "PID controller")
      "Enumeration defining P, PI, PD, or PID simple controller type" annotation (
       Evaluate=true);

    type AnalogFilter = enumeration(
        CriticalDamping "Filter with critical damping",
        Bessel "Bessel filter",
        Butterworth "Butterworth filter",
        ChebyshevI "Chebyshev I filter")
      "Enumeration defining the method of filtering" annotation (Evaluate=true);

    type FilterType = enumeration(
        LowPass "Low pass filter",
        HighPass "High pass filter",
        BandPass "Band pass filter",
        BandStop "Band stop / notch filter")
      "Enumeration of analog filter types (low, high, band pass or band stop filter)"
      annotation (Evaluate=true);

    type Regularization = enumeration(
        Exp "Exponential regularization (smooth)",
        Sine "Sinusoidal regularization (smooth 1st derivative)",
        Linear "Linear regularization",
        Cosine "Cosine regularization")
      "Enumeration defining the regularization around zero";

    type LimiterHomotopy = enumeration(
        NoHomotopy "Homotopy is not used",
        Linear "Simplified model without limits",
        UpperLimit "Simplified model fixed at upper limit",
        LowerLimit "Simplified model fixed at lower limit")
      "Enumeration defining use of homotopy in limiter components" annotation (Evaluate=true);

    type VariableLimiterHomotopy = enumeration(
        NoHomotopy "Simplified model = actual model",
        Linear "Simplified model: y = u",
        Fixed "Simplified model: y = ySimplified")
      "Enumeration defining use of homotopy in variable limiter components" annotation (Evaluate=true);

    class ExternalCombiTimeTable
      "External object of 1-dim. table where first column is time"
      extends ExternalObject;

      function constructor "Initialize 1-dim. table where first column is time"
        extends Modelica.Icons.Function;
        input String tableName "Table name";
        input String fileName "File name";
        input Real table[:, :];
        input SI.Time startTime;
        input Integer columns[:];
        input Blocks.Types.Smoothness smoothness;
        input Blocks.Types.Extrapolation extrapolation;
        input SI.Time shiftTime=0.0;
        input Blocks.Types.TimeEvents timeEvents=Blocks.Types.TimeEvents.Always;
        input Boolean verboseRead=true "= true: Print info message; = false: No info message";
        output ExternalCombiTimeTable externalCombiTimeTable;
      external "C" externalCombiTimeTable = ModelicaStandardTables_CombiTimeTable_init2(
              fileName,
              tableName,
              table,
              size(table, 1),
              size(table, 2),
              startTime,
              columns,
              size(columns, 1),
              smoothness,
              extrapolation,
              shiftTime,
              timeEvents,
              verboseRead) annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end constructor;

      function destructor "Terminate 1-dim. table where first column is time"
        extends Modelica.Icons.Function;
        input ExternalCombiTimeTable externalCombiTimeTable;
      external "C" ModelicaStandardTables_CombiTimeTable_close(
          externalCombiTimeTable) annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end destructor;

    end ExternalCombiTimeTable;

    class ExternalCombiTable1D
      "External object of 1-dim. table defined by matrix"
      extends ExternalObject;

      function constructor "Initialize 1-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input String tableName "Table name";
        input String fileName "File name";
        input Real table[:, :];
        input Integer columns[:];
        input Blocks.Types.Smoothness smoothness;
        input Blocks.Types.Extrapolation extrapolation=Blocks.Types.Extrapolation.LastTwoPoints;
        input Boolean verboseRead=true "= true: Print info message; = false: No info message";
        output ExternalCombiTable1D externalCombiTable1D;
      external "C" externalCombiTable1D = ModelicaStandardTables_CombiTable1D_init2(
              fileName,
              tableName,
              table,
              size(table, 1),
              size(table, 2),
              columns,
              size(columns, 1),
              smoothness,
              extrapolation,
              verboseRead) annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end constructor;

      function destructor "Terminate 1-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input ExternalCombiTable1D externalCombiTable1D;
      external "C" ModelicaStandardTables_CombiTable1D_close(externalCombiTable1D)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end destructor;

    end ExternalCombiTable1D;

    class ExternalCombiTable2D
      "External object of 2-dim. table defined by matrix"
      extends ExternalObject;

      function constructor "Initialize 2-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input String tableName "Table name";
        input String fileName "File name";
        input Real table[:, :];
        input Blocks.Types.Smoothness smoothness;
        input Blocks.Types.Extrapolation extrapolation=Blocks.Types.Extrapolation.LastTwoPoints;
        input Boolean verboseRead=true "= true: Print info message; = false: No info message";
        output ExternalCombiTable2D externalCombiTable2D;
      external "C" externalCombiTable2D = ModelicaStandardTables_CombiTable2D_init2(
              fileName,
              tableName,
              table,
              size(table, 1),
              size(table, 2),
              smoothness,
              extrapolation,
              verboseRead) annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end constructor;

      function destructor "Terminate 2-dim. table defined by matrix"
        extends Modelica.Icons.Function;
        input ExternalCombiTable2D externalCombiTable2D;
      external "C" ModelicaStandardTables_CombiTable2D_close(externalCombiTable2D)
          annotation (IncludeDirectory="modelica://Modelica/Resources/C-Sources", Include="#include \"ModelicaStandardTables.h\"", Library={"ModelicaStandardTables", "ModelicaIO", "ModelicaMatIO", "zlib"});
      end destructor;

    end ExternalCombiTable2D;
    annotation (Documentation(info="<html>
<p>
In this package <strong>types</strong>, <strong>constants</strong> and <strong>external objects</strong> are defined that are used
in library Modelica.Blocks. The types have additional annotation choices
definitions that define the menus to be built up in the graphical
user interface when the type is used as parameter in a declaration.
</p>
</html>"));
  end Types;

  package Icons "Icons for Blocks"
      extends Modelica.Icons.IconsPackage;
      partial block Block "Basic graphical layout of input/output block"

        annotation (
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255})}),
        Documentation(info="<html>
<p>
Block that has only the basic icon for an input/output
block (no declarations, no equations). Most blocks
of package Modelica.Blocks inherit directly or indirectly
from this block.
</p>
</html>"));

      end Block;

      partial block BooleanBlock "Basic graphical layout of Boolean block"

        annotation (
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={255,0,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255})}),
        Documentation(info="<html>
<p>
Block that has only the basic icon for an input/output,
Boolean block (no declarations, no equations).
</p>
</html>"));

      end BooleanBlock;

      partial block DiscreteBlock
      "Graphical layout of discrete block component icon"

        annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={0,0,127},
              fillColor={223,211,169},
              borderPattern=BorderPattern.Raised,
              fillPattern=FillPattern.Solid), Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255})}),
                             Documentation(info="<html>
<p>
Block that has only the basic icon for an input/output,
discrete block (no declarations, no equations), e.g.,
from Blocks.Discrete.
</p>
</html>"));
      end DiscreteBlock;

  partial block IntegerBlock "Basic graphical layout of Integer block"

    annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={Rectangle(
              extent={{-100,-100},{100,100}},
              lineColor={255,127,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid), Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255})}),
          Documentation(info="<html>
<p>
Block that has only the basic icon for an input/output,
Integer block (no declarations, no equations).
</p>
</html>"));
  end IntegerBlock;

    partial block PartialBooleanBlock "Basic graphical layout of logical block"

      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                -100},{100,100}}), graphics={Rectangle(
              extent={{-100,100},{100,-100}},
              fillColor={210,210,210},
              fillPattern=FillPattern.Solid,
              borderPattern=BorderPattern.Raised), Text(
              extent={{-150,150},{150,110}},
              textString="%name",
              textColor={0,0,255})}), Documentation(info="<html>
<p>
Block that has only the basic icon for an input/output,
Boolean block (no declarations, no equations) used especially
in the Blocks.Logical library.
</p>
</html>"));
    end PartialBooleanBlock;
  end Icons;
annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
      Rectangle(
        origin={0.0,35.1488},
        fillColor={255,255,255},
        extent={{-30.0,-20.1488},{30.0,20.1488}}),
      Rectangle(
        origin={0.0,-34.8512},
        fillColor={255,255,255},
        extent={{-30.0,-20.1488},{30.0,20.1488}}),
      Line(
        origin={-51.25,0.0},
        points={{21.25,-35.0},{-13.75,-35.0},{-13.75,35.0},{6.25,35.0}}),
      Polygon(
        origin={-40.0,35.0},
        pattern=LinePattern.None,
        fillPattern=FillPattern.Solid,
        points={{10.0,0.0},{-5.0,5.0},{-5.0,-5.0}}),
      Line(
        origin={51.25,0.0},
        points={{-21.25,35.0},{13.75,35.0},{13.75,-35.0},{-6.25,-35.0}}),
      Polygon(
        origin={40.0,-35.0},
        pattern=LinePattern.None,
        fillPattern=FillPattern.Solid,
        points={{-10.0,0.0},{5.0,5.0},{5.0,-5.0}})}), Documentation(info="<html>
<p>
This library contains input/output blocks to build up block diagrams.
</p>

<dl>
<dt><strong>Main Author:</strong></dt>
<dd><a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a><br>
    Deutsches Zentrum f&uuml;r Luft und Raumfahrt e. V. (DLR)<br>
    Oberpfaffenhofen<br>
    Postfach 1116<br>
    D-82230 Wessling<br>
    email: <a href=\"mailto:Martin.Otter@dlr.de\">Martin.Otter@dlr.de</a><br></dd>
</dl>
<p>
Copyright &copy; 1998-2020, Modelica Association and contributors
</p>
</html>", revisions="<html>
<ul>
<li><em>June 23, 2004</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Introduced new block connectors and adapted all blocks to the new connectors.
       Included subpackages Continuous, Discrete, Logical, Nonlinear from
       package ModelicaAdditions.Blocks.
       Included subpackage ModelicaAdditions.Table in Modelica.Blocks.Sources
       and in the new package Modelica.Blocks.Tables.
       Added new blocks to Blocks.Sources and Blocks.Logical.
       </li>
<li><em>October 21, 2002</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>
       and Christian Schweiger:<br>
       New subpackage Examples, additional components.
       </li>
<li><em>June 20, 2000</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a> and
       Michael Tiller:<br>
       Introduced a replaceable signal type into
       Blocks.Interfaces.RealInput/RealOutput:
<blockquote><pre>
replaceable type SignalType = Real
</pre></blockquote>
       in order that the type of the signal of an input/output block
       can be changed to a physical type, for example:
<blockquote><pre>
Sine sin1(outPort(redeclare type SignalType=Modelica.Units.SI.Torque))
</pre></blockquote>
      </li>
<li><em>Sept. 18, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Renamed to Blocks. New subpackages Math, Nonlinear.
       Additional components in subpackages Interfaces, Continuous
       and Sources.</li>
<li><em>June 30, 1999</em>
       by <a href=\"http://www.robotic.dlr.de/Martin.Otter/\">Martin Otter</a>:<br>
       Realized a first version, based on an existing Dymola library
       of Dieter Moormann and Hilding Elmqvist.</li>
</ul>
</html>"),
    uses(Modelica(version="4.0.0")));
end Blocks;
