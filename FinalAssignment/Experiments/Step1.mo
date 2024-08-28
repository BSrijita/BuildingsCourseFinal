within FinalAssignment.Experiments;
model Step1 "No Ventilation"
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
    redeclare package Medium = Buildings.Media.Air) annotation (Placement(transformation(extent={{28,-6},{56,22}})));
  Modelica.Blocks.Sources.CombiTimeTable SenGai(
    table=[0*24 + 0,5; 0*24 + 18,2; 0*24 + 24,5],
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic,
    timeScale(displayUnit="h") = 3600)          annotation (Placement(transformation(extent={{-60,8},{-40,28}})));
  Modelica.Blocks.Sources.Constant LatGai(k=0) annotation (Placement(transformation(extent={{-58,-44},{-38,-24}})));
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=ModelicaServices.ExternalReferences.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")) annotation (Placement(transformation(extent={{-18,42},{10,66}})));
equation
  connect(LatGai.y, zonHVAC.intLatGai) annotation (Line(points={{-37,-34},{16,-34},{16,12},{26,12}}, color={0,0,127}));
  connect(weaDat.weaBus, zonHVAC.weaBus) annotation (Line(
      points={{10,54},{52,54},{52,19}},
      color={255,204,51},
      thickness=0.5));
  connect(SenGai.y[1], zonHVAC.intSenGai) annotation (Line(points={{-39,18},{26,18}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=2678400,
      Interval=3600,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"));
end Step1;
