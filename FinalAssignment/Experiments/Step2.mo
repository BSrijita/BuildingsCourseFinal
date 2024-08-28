within FinalAssignment.Experiments;
model Step2 "With Ventilation"
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=ModelicaServices.ExternalReferences.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    annotation (Placement(transformation(extent={{-94,74},{-74,94}})));
  Modelica.Blocks.Sources.Constant latGai(k=0)
    annotation (Placement(transformation(extent={{6,-48},{26,-28}})));
  Buildings.Fluid.Movers.FlowControlled_m_flow fan(
    redeclare package Medium = Buildings.Media.Air,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    nominalValuesDefineDefaultPressureCurve=true,
    m_flow_nominal=0.05,
    addPowerToMedium=false)
    annotation (Placement(transformation(extent={{-26,12},{-6,32}})));
  Buildings.Fluid.Sources.Outside out(redeclare package Medium =
        Buildings.Media.Air, nPorts=2)
    annotation (Placement(transformation(extent={{-90,-18},{-70,2}})));
  Modelica.Blocks.Sources.Constant airFlow(k=0.05)
    annotation (Placement(transformation(extent={{-60,38},{-40,58}})));
  Buildings.ThermalZones.ISO13790.Zone5R1C.ZoneHVAC zonHVAC(
    airRat=0.41,
    AWin={0,0,12,6},
    UWin=0.9,
    AWal={36,30,24,24},
    ARoo=120,
    UWal=0.25,
    URoo=0.15,
    UFlo=0.11,
    b=0.5,
    AFlo=120,
    VRoo=360,
    hInt=3.45,
    redeclare Buildings.ThermalZones.ISO13790.Data.Medium buiMas,
    surTil={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949},
    surAzi={3.1415926535898,-1.5707963267949,0,1.5707963267949},
    winFra=0.01,
    gFac=0.5,
    redeclare package Medium = Buildings.Media.Air,
    nPorts=2)                                       annotation (Placement(transformation(extent={{54,6},{82,34}})));
  Modelica.Blocks.Sources.CombiTimeTable SenGai(
    table=[0,5*120; 8,2*120; 18,5*120; 24,5*120],
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    timeScale(displayUnit="h") = 3600)          annotation (Placement(transformation(extent={{4,44},{
            24,64}})));
equation
  connect(out.ports[1],fan. port_a) annotation (Line(points={{-70,-9},{-32,-9},{-32,22},{-26,22}},
                              color={0,127,255}));
  connect(fan.m_flow_in, airFlow.y)
    annotation (Line(points={{-16,34},{-16,48},{-39,48}}, color={0,0,127}));
  connect(out.weaBus, weaDat.weaBus) annotation (Line(
      points={{-90,-7.8},{-94,-7.8},{-94,60},{-74,60},{-74,84}},
      color={255,204,51},
      thickness=0.5));
  connect(zonHVAC.weaBus, weaDat.weaBus) annotation (Line(
      points={{78,31},{78,84},{-74,84}},
      color={255,204,51},
      thickness=0.5));
  connect(latGai.y, zonHVAC.intLatGai) annotation (Line(points={{27,-38},{42,-38},{42,24},{52,24}}, color={0,0,127}));
  connect(fan.port_b, zonHVAC.ports[1]) annotation (Line(points={{-6,22},{44,22},{44,10.825},{55,10.825}}, color={0,127,255}));
  connect(out.ports[2], zonHVAC.ports[2]) annotation (Line(points={{-70,-7},{-70,-10},{-32,-10},{-32,8},{40,8},{40,22},{44,22},{44,10},{55,10},{55,12.775}}, color={0,127,255}));
  connect(SenGai.y[1], zonHVAC.intSenGai) annotation (Line(points={{25,54},{36,
          54},{36,30},{52,30}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}})),
    experiment(
      StopTime=2678400,
      Interval=3600,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end Step2;
