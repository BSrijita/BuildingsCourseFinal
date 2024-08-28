within FinalAssignment.Experiments;
model Step3 "With Ventilation and Heating System"
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
        ModelicaServices.ExternalReferences.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Buildings.Fluid.Movers.Preconfigured.FlowControlled_m_flow
                                               fan(
    redeclare package Medium = Buildings.Media.Air,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=0.05,
    addPowerToMedium=false)
    annotation (Placement(transformation(extent={{14,4},{34,24}})));
  Buildings.Fluid.Sources.Outside out(redeclare package Medium =
        Buildings.Media.Air, nPorts=2)
    annotation (Placement(transformation(extent={{-64,4},{-44,24}})));
  Modelica.Blocks.Sources.Constant airFlow(k=0.05)
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));
  Buildings.Fluid.HeatExchangers.Heater_T Boiler(
    redeclare package Medium = Buildings.Media.Water,
    m_flow_nominal=0.1,
    dp_nominal=0,
    QMax_flow(displayUnit="W") = 6000)  annotation (Placement(transformation(extent={{112,-110},{132,-90}})));
  Buildings.Controls.Continuous.LimPID conPID(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.1,
    Ti=120,
    Td=200,
    yMax=2,
    reverseActing=true)
    annotation (Placement(transformation(extent={{-28,-80},{-8,-60}})));
  Modelica.Blocks.Sources.CombiTimeTable setPointAir(
    table=[0.0,273.15 + 20],
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    annotation (Placement(transformation(extent={{-72,-80},{-52,-60}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = Buildings.Media.Air, m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{68,4},{88,24}})));
  Buildings.Fluid.Movers.Preconfigured.FlowControlled_m_flow
                                               pump(
    redeclare package Medium = Buildings.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    m_flow_nominal=0.1,
    addPowerToMedium=false) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={144,-70})));
  Modelica.Blocks.Sources.Constant supWatTem(k=273.15 + 70)
    annotation (Placement(transformation(extent={{22,-102},{42,-82}})));
  Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium =
        Buildings.Media.Water, nPorts=1)
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
  Modelica.Blocks.Continuous.Integrator HeatEnergy(k=1/3600000)
    annotation (Placement(transformation(extent={{168,-102},{188,-82}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTem1(redeclare package Medium =
        Buildings.Media.Water, m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={144,-36})));
  Buildings.ThermalZones.ISO13790.Zone5R1C.ZoneHVAC zonHVAC(
    airRat=0.4,
    AWin={0,0,12,6},
    UWin=0.9,
    AWal={36,30,24,24},
    ARoo=120,
    UWal=0.25,
    URoo=0.15,
    UFlo=0.11,
    AFlo=120,
    VRoo=360,
    winFra=0.01,
    gFac=0.5,
    redeclare package Medium = Buildings.Media.Air,
    redeclare Buildings.ThermalZones.ISO13790.Data.Medium buiMas,
    surTil={1.5707963267949,1.5707963267949,1.5707963267949,1.5707963267949},
    surAzi={3.1415926535898,-1.5707963267949,0,1.5707963267949},
    nPorts=2)
    annotation (Placement(transformation(extent={{168,8},{206,40}})));
  Modelica.Blocks.Sources.Constant latGai(k=0)
    annotation (Placement(transformation(extent={{126,34},{146,54}})));
  Modelica.Blocks.Sources.CombiTimeTable SenGai(
    table=[0*24 + 0,5; 0*24 + 18,2; 0*24 + 24,5],
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    timeScale(displayUnit="h") = 3600)          annotation (Placement(transformation(extent={{126,64},{146,84}})));
  Components.Radiator             radiator(ReturnWaterTemp(k=273.15 + 50), Rad(m_flow_nominal=0.1, QMin_flow(displayUnit="W") = -230))
                                                                                                                                  annotation (Placement(transformation(extent={{130,-20},{110,-4}})));
equation
  connect(out.ports[1],fan. port_a)
    annotation (Line(points={{-44,13},{-14,13},{-14,14},{14,14}},
                                                   color={0,127,255}));
  connect(airFlow.y, fan.m_flow_in)
    annotation (Line(points={{11,60},{24,60},{24,26}},    color={0,0,127}));
  connect(supWatTem.y, Boiler.TSet) annotation (Line(points={{43,-92},{110,-92}}, color={0,0,127}));
  connect(Boiler.Q_flow, HeatEnergy.u) annotation (Line(points={{133,-92},{166,-92}}, color={0,0,127}));
  connect(zonHVAC.intLatGai, latGai.y) annotation (Line(points={{165.286,28.5714},{154,28.5714},{154,44},{147,44}},
                                                  color={0,0,127}));
  connect(weaDat.weaBus, out.weaBus) annotation (Line(
      points={{-80,90},{-74,90},{-74,14.2},{-64,14.2}},
      color={255,204,51},
      thickness=0.5));
  connect(zonHVAC.weaBus, weaDat.weaBus) annotation (Line(
      points={{200.571,36.5714},{200.571,102},{-70,102},{-70,90},{-80,90}},
      color={255,204,51},
      thickness=0.5));
  connect(conPID.u_s, setPointAir.y[1]) annotation (
    Line(points={{-30,-70},{-51,-70}},      color = {0, 0, 127}));
  connect(SenGai.y[1], zonHVAC.intSenGai) annotation (Line(points={{147,74},{156,74},{156,35.4286},{165.286,35.4286}}, color={0,0,127}));
  connect(fan.port_b, senTem.port_a) annotation (Line(points={{34,14},{68,14}}, color={0,127,255}));
  connect(radiator.port_house, zonHVAC.heaPorAir) annotation (Line(points={{120,-4},{120,90},{192.429,90},{192.429,33.1429}}, color={191,0,0}));
  connect(senTem.port_b, zonHVAC.ports[1]) annotation (Line(points={{88,14},{100,13.5143},{169.357,13.5143}}, color={0,127,255}));
  connect(conPID.u_m, zonHVAC.TAir) annotation (Line(points={{-18,-82},{-18,-120},{222,-120},{222,33.1429},{207.357,33.1429}}, color={0,0,127}));
  connect(out.ports[2], zonHVAC.ports[2]) annotation (Line(points={{-44,15},{-38,15},{-38,-6},{108,-6},{108,15.7429},{169.357,15.7429}}, color={0,127,255}));
  connect(Boiler.port_b, pump.port_a) annotation (Line(points={{132,-100},{144,-100},{144,-80}}, color={0,127,255}));
  connect(senTem1.port_a, pump.port_b) annotation (Line(points={{144,-46},{144,-60}}, color={0,127,255}));
  connect(radiator.port_supply, senTem1.port_b) annotation (Line(points={{130.2,-14.2},{144,-14.2},{144,-26}}, color={0,127,255}));
  connect(radiator.port_return, Boiler.port_a) annotation (Line(points={{110,-13.8},{102,-13.8},{102,-14},{98,-14},{98,-100},{112,-100}}, color={0,127,255}));
  connect(bou.ports[1], Boiler.port_a) annotation (Line(points={{80,-50},{98,-50},{98,-100},{112,-100}}, color={0,127,255}));
  connect(conPID.y, pump.m_flow_in) annotation (Line(points={{-7,-70},{132,-70}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{240,120}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{240,120}})),
    experiment(
      StopTime=2678400,
      Interval=3600,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end Step3;
