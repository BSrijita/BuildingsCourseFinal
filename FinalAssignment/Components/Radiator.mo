within FinalAssignment.Components;
model Radiator
  Modelica.Fluid.Interfaces.FluidPort_a port_supply(redeclare package Medium = Buildings.Media.Water) annotation (Placement(transformation(extent={{-118,-16},{-86,16}}), iconTransformation(extent={{-112,-12},{-92,8}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_return(redeclare package Medium = Buildings.Media.Water) annotation (Placement(transformation(extent={{82,-16},{116,16}}), iconTransformation(extent={{90,-8},{110,12}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_house annotation (Placement(transformation(extent={{-14,84},{14,112}}), iconTransformation(extent={{-10,90},{10,110}})));
  Modelica.Blocks.Sources.Constant ReturnWaterTemp(k=273.15 + 50) annotation (Placement(transformation(extent={{-70,42},{-50,62}})));
  Modelica.Blocks.Math.Gain gain(k=-1) annotation (Placement(transformation(extent={{22,42},{42,62}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow HeatToHouse annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={64,76})));
  Buildings.Fluid.HeatExchangers.SensibleCooler_T Rad(
    redeclare package Medium = Buildings.Media.Water,
    m_flow_nominal=0.05,
    dp_nominal=0,
    QMin_flow=-6000)
                    annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
equation
  connect(port_supply, Rad.port_a) annotation (Line(points={{-102,0},{-12,0}}, color={0,127,255}));
  connect(Rad.port_b, port_return) annotation (Line(points={{8,0},{99,0}}, color={0,127,255}));
  connect(ReturnWaterTemp.y, Rad.TSet) annotation (Line(points={{-49,52},{-40,52},{-40,8},{-14,8}}, color={0,0,127}));
  connect(Rad.Q_flow, gain.u) annotation (Line(points={{9,8},{14,8},{14,52},{20,52}}, color={0,0,127}));
  connect(gain.y, HeatToHouse.Q_flow) annotation (Line(points={{43,52},{48,52},{48,76},{54,76}}, color={0,0,127}));
  connect(HeatToHouse.port, port_house) annotation (Line(points={{74,76},{82,76},{82,88},{0,88},{0,98}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},{100,100}}), graphics={
        Rectangle(
          extent={{-80,64},{80,-60}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillPattern=FillPattern.Solid,
          fillColor={95,95,95},
          pattern=LinePattern.None),
        Rectangle(
          extent={{-80,80},{-60,-80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-34,80},{-14,-80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{14,80},{34,-80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{60,80},{80,-80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=0.5,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-60},{100,100}})));
end Radiator;
