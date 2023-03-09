using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
//using ISAAR.MSolve.Analyzers;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.NonLinear;
//using ISAAR.MSolve.Analyzers.NonLinear;
//using ISAAR.MSolve.Discretization;
using MGroup.NumericalAnalyzers.Discretization;
using MGroup.FEM;
using MGroup.FEM.Structural;
using MGroup.FEM.Structural.Tests;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.FEM.Structural.Tests.Integration;
using MGroup.Solvers;
//using ISAAR.MSolve.FEM;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.MSolve.Discretization.Entities;
using System.Data;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Transient;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Discretization;
using MGroup.Environments;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Numerics.Interpolation;
using System.IO;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Constitutive;
using MGroup.MSolve.DataStructures;
using MGroup.Solvers.AlgebraicModel;
using MGroup.MSolve.Solution.LinearSystem;
using IntelMKL.LP64;
//using ISAAR.MSolve.PreProcessor;
//using ISAAR.MSolve.Discretization.Commons;
//using ISAAR.MSolve.Discretization.FreedomDegrees;
//using ISAAR.MSolve.Discretization.Integration.Quadratures;
//using ISAAR.MSolve.FEM.Elements;
//using ISAAR.MSolve.FEM.Entities;
//using ISAAR.MSolve.Logging;
//using ISAAR.MSolve.Materials;
//using ISAAR.MSolve.Problems;
//using ISAAR.MSolve.Solvers.Direct;
//using ISAAR.MSolve.PreProcessor.Materials;
//using ISAAR.MSolve.FEM.Entities.TemporalFunctions;


namespace MGroup.FEM.Structural.Tests.ExampleModels
{
    public class VonMisesNonLinearTestExample
    {
        //public static Model model = new Model();
        //public static Model CreateModel()
        //{
        //    MakeHexaSoil(model);
        //    return model;
        //}

        public static double startX = 0.0;
        public static double startY = 0.0;
        public static double startZ = 0.0;
        public static double LengthX = 1.0;
        public static double LengthY = 1.0;
        public static double LengthZ = 1.0;
        int nodeID = 1;
        public static double hx = 2.0;
        public static double hy = 2.0;
        public static double hz = 2.0;
        public static int imax = (int)Math.Truncate(hx / LengthX) + 1;
        public static int jmax = (int)Math.Truncate(hy / LengthY) + 1;
        public static int kmax = (int)Math.Truncate(hz / LengthZ) + 1;

        public static void MakeHexaSoil(Model model)
        {
            // xreiazetai na rythmizei kaneis ta megethi me auto to configuration
            int nodeID = 1;
            for (int l = 0; l < kmax; l++)
            {
                for (int k = 0; k < jmax; k++)
                {
                    for (int j = 0; j < imax; j++)
                    {
                        model.NodesDictionary.Add(nodeID, new Node(nodeID, startX + j * LengthX, startY + k * LengthY, startZ + l * LengthZ));
                        nodeID++;
                    }
                }
            }
            //nodeID = 1;
            var constraints = new List<INodalDisplacementBoundaryCondition>();
            constraints.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[6], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[7], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[8], StructuralDof.TranslationZ, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationX, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationY, amount: 0d));
            constraints.Add(new NodalDisplacement(model.NodesDictionary[9], StructuralDof.TranslationZ, amount: 0d));


            //for (int j = 0; j < jmax; j++)
            //{
            //    for (int k = 0; k < imax; k++)
            //    {

            //        constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationX, amount: 0d));
            //        constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationY, amount: 0d));
            //        constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationZ, amount: 0d));
            //        nodeID++;
            //    }
            //}


            //model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
            //    constraints, new NodalLoad[] { }));


            //model.NodesDictionary[5].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
            //model.NodesDictionary[6].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
            //model.NodesDictionary[7].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });
            //model.NodesDictionary[8].Constraints.Add(new Constraint { DOF = StructuralDof.TranslationY });

            ElasticMaterial3D material1 = new ElasticMaterial3D(800000d, 1.0d / 3.0d);

            //ContinuumElement3DFactory e1;

            int IDhelp = 1;
            var elementCounter = 1;
            for (int ii = 1; ii < model.NodesDictionary.Count + 1; ii++)
            {
                var nodecheck = model.NodesDictionary[ii];
                if (nodecheck.X != hx && nodecheck.Y != hy && nodecheck.Z != hz)
                {
                    const int gpNo = 8;
                    //var initialStresses = new double[6];
                    var element1Nodes = new INode[8];
                    //element1Nodes[6] = model.NodesDictionary[ii];
                    //element1Nodes[7] = model.NodesDictionary[ii + 1];
                    //element1Nodes[4] = model.NodesDictionary[ii + 1 + imax];
                    //element1Nodes[5] = model.NodesDictionary[ii + imax];
                    //element1Nodes[2] = model.NodesDictionary[ii + jmax * imax];
                    //element1Nodes[3] = model.NodesDictionary[ii + jmax * imax + 1];
                    //element1Nodes[0] = model.NodesDictionary[ii + jmax * imax + 1 + imax];
                    //element1Nodes[1] = model.NodesDictionary[ii + jmax * imax + imax];

                    element1Nodes[0] = model.NodesDictionary[ii];
                    element1Nodes[1] = model.NodesDictionary[ii + 1];
                    element1Nodes[2] = model.NodesDictionary[ii + 1 + imax];
                    element1Nodes[3] = model.NodesDictionary[ii + imax];
                    element1Nodes[4] = model.NodesDictionary[ii + jmax * imax];
                    element1Nodes[5] = model.NodesDictionary[ii + jmax * imax + 1];
                    element1Nodes[6] = model.NodesDictionary[ii + jmax * imax + 1 + imax];
                    element1Nodes[7] = model.NodesDictionary[ii + jmax * imax + imax];

                    
                    var nodeCoordinates = new double[8, 3];
                    for (int i = 0; i < 8; i++)
                    {
                        nodeCoordinates[i, 0] = element1Nodes[i].X;
                        nodeCoordinates[i, 1] = element1Nodes[i].Y;
                        nodeCoordinates[i, 2] = element1Nodes[i].Z;
                    }
                    //var gaussPointMaterials = new MohrCoulombMaterialNLH[8];
                    //double young = 800000;
                    //double poisson = 1.0 / 3.0;
                    //double friction = 30 * Math.PI / 180;
                    //double dilation = 30 * Math.PI / 180;
                    //double cohesion = 150;
                    //for (int i = 0; i < gpNo; i++)
                    //    gaussPointMaterials[i] = new MohrCoulombMaterialNLH(young, poisson, cohesion, friction, dilation);
                    var gaussPointMaterials = new VonMises3DNonLinearHardening[8];
                    double young = 800000d;
                    double poisson = 1.0d / 3.0d;
                    double syield = 5000d;
                    for (int i = 0; i < gpNo; i++)
                        gaussPointMaterials[i] = new VonMises3DNonLinearHardening(young, poisson, syield);
                    //var gaussPointMaterials = new DruckerPragerNLH[8];
                    //double young = 800000;
                    //double poisson = 1.0 / 3.0;
                    //double friction = 30 * Math.PI / 180;
                    //double dilation = 30 * Math.PI / 180;
                    //double cohesion = 150;
                    //for (int i = 0; i < gpNo; i++)
                    //    gaussPointMaterials[i] = new DruckerPragerNLH(young, poisson, cohesion, friction, dilation, "Outer Cone");
                    //var gaussPointMaterials = new Tresca[8];
                    //double young = 800000;
                    //double poisson = 1.0 / 3.0;
                    //double syield = 300;
                    //for (int i = 0; i < gpNo; i++)
                    //    gaussPointMaterials[i] = new Tresca(young, poisson, syield);
                    //var gaussPointMaterials = new TsaiHill[8];
                    //double[] youngmouduli = new double[6];
                    //youngmouduli[0] = 180000000;
                    //youngmouduli[1] = 25000000;
                    //youngmouduli[2] = 15000000;
                    //youngmouduli[3] = 7500000;
                    //youngmouduli[4] = 5000000;
                    //youngmouduli[5] = 10000000;
                    //double[] poissonratioi = new double[3];
                    //poissonratioi[0] = 0.35;
                    //poissonratioi[1] = 0.33;
                    //poissonratioi[2] = 0.28;
                    //double[] sigmas = new double[6];
                    //sigmas[0] = 1100000;
                    //sigmas[1] = 620000;
                    //sigmas[2] = 580000;
                    //sigmas[3] = 45000;
                    //sigmas[4] = 82000;
                    //sigmas[5] = 52000;
                    //for (int i = 0; i < gpNo; i++)
                    //    gaussPointMaterials[i] = new TsaiHill(youngmouduli, poissonratioi, sigmas);
                    //var gaussPointMaterials = new TsaiWu[8];
                    //double[] youngmouduli = new double[6];
                    //youngmouduli[0] = 180000000;
                    //youngmouduli[1] = 25000000;
                    //youngmouduli[2] = 15000000;
                    //youngmouduli[3] = 7500000;
                    //youngmouduli[4] = 5000000;
                    //youngmouduli[5] = 10000000;
                    //double[] poissonratioi = new double[3];
                    //poissonratioi[0] = 0.35;
                    //poissonratioi[1] = 0.33;
                    //poissonratioi[2] = 0.28;
                    //double[] sigmas = new double[12];
                    //sigmas[0] = 1100000;
                    //sigmas[1] = 620000;
                    //sigmas[2] = 580000;
                    //sigmas[3] = 45000;
                    //sigmas[4] = 82000;
                    //sigmas[5] = 52000;
                    //sigmas[6] = 900000;
                    //sigmas[7] = 540000;
                    //sigmas[8] = 470000;
                    //sigmas[9] = 0.5;
                    //sigmas[10] = 0.5;
                    //sigmas[11] = 0.5;
                    //for (int i = 0; i < gpNo; i++)
                    //    gaussPointMaterials[i] = new TsaiWu(youngmouduli, poissonratioi, sigmas);
                    //var gaussPointMaterials = new KavvadasClays[8];
                    //double compressibilityfactor = 0.5*0.008686;
                    //double criticalstateline = 0.733609251;
                    //double alpha = 1;
                    // double ksi = 0.05;
                    // double[] initialstresses = new double[6];
                    // initialstresses[0] = -1000;
                    // initialstresses[1] = -1000;
                    // initialstresses[2] = -1000;
                    // initialstresses[3] = 0;
                    // initialstresses[4] = 0;
                    //  initialstresses[5] = 0;
                    //  double Htot = hz;
                    // for (int i = 0; i < gpNo; i++)
                    //    gaussPointMaterials[i] = new KavvadasClays(compressibilityfactor, criticalstateline, alpha, ksi, initialstresses, Htot);
                    var elementFactory = new ContinuumElement3DFactory(new VonMises3DNonLinearHardening(800000d, 1.0d / 3.0d, 5000d), new TransientAnalysisProperties(1, 0, 0));
                    var element = elementFactory.CreateElement(CellType.Hexa8,
                        new INode[]
                        {
                            element1Nodes[6],
                            element1Nodes[7],
                            element1Nodes[4],
                            element1Nodes[5],
                            element1Nodes[2],
                            element1Nodes[3],
                            element1Nodes[0],
                            element1Nodes[1]

                        });
                    //elementType1.Density = 25.0;
                    //elementType1.RayleighAlpha = 0.35;
                    //elementType1.RayleighBeta = 0.003;
                    //e1.ElementType = elementType1; //yliko meta diorthosi to material1
                    //var element1Nodes = new INode[8];

                    //element1Nodes[6] = model.NodesDictionary[ii];
                    //element1Nodes[7] = model.NodesDictionary[ii + 1];
                    //element1Nodes[5] = model.NodesDictionary[ii + 1 + imax];
                    //element1Nodes[4] = model.NodesDictionary[ii + imax];
                    //element1Nodes[2] = model.NodesDictionary[ii + jmax * imax];
                    //element1Nodes[3] = model.NodesDictionary[ii + jmax * imax + 1];
                    //element1Nodes[1] = model.NodesDictionary[ii + jmax * imax + 1 + imax];
                    //element1Nodes[0] = model.NodesDictionary[ii + jmax * imax + imax];

                    {
                        int ID = IDhelp;
                    };
                    IDhelp++;
                    int subdomainID = 1;
                    element.ID = elementCounter;
                    elementCounter += 1;
                    //element.Node
                    model.ElementsDictionary.Add(element.ID, element);
                    model.SubdomainsDictionary[subdomainID].Elements.Add(element);
                }
            };

            //DOFType doftype1;
            //doftype1 = new DOFType();
            #region initialloads
            var loads = new List<INodalLoadBoundaryCondition>();
            //foreach (Node nodecheck in model.NodesDictionary.Values)
            //{

            //    var bool1 = (nodecheck.X == hx / 2) && (nodecheck.Y == hy / 2) && (nodecheck.Z == hz);

            //        if (bool1)

            //        {

            //            loads.Add(new NodalLoad(model.NodesDictionary[nodeID], StructuralDof.TranslationZ, amount: 1 * (-15000.0d)));
            //        }

            //        var bool2 = (nodecheck.X == 0 || nodecheck.X == hx) && (nodecheck.Y == 0 || nodecheck.Y == hy) && (nodecheck.Z == hz);
            //        if (bool2)
            //        {
            //            loads.Add(new NodalLoad(model.NodesDictionary[nodeID], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));
            //        }

            //        var bool3 = (nodecheck.Z == hz) && (!bool1) && (!bool2);

            //        if (bool3)
            //        {
            //            loads.Add(new NodalLoad(model.NodesDictionary[nodeID], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            //        }



            //}
            loads.Add(new NodalLoad(model.NodesDictionary[19], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[22], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[25], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[20], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[23], StructuralDof.TranslationZ, amount: 1 * (-15000.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[26], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[21], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[24], StructuralDof.TranslationZ, amount: 1 * (-7500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[27], StructuralDof.TranslationZ, amount: 1 * (-3750.0d)));


            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));


            #endregion

        }

        //public static double[] RetrieveDisplacementsOfFreeDofs(GlobalAlgebraicModel<SkylineMatrix> globalAlgebraicModel, IGlobalVector uInitialFreeDOFDisplacementsPerSubdomain)
        //{
        //    var uInitialFreeDOFs_state1_Data = globalAlgebraicModel.ExtractAllResults(uInitialFreeDOFDisplacementsPerSubdomain);
        //    double[] uInitialFreeDOFs_state1_array = new double[globalAlgebraicModel.SubdomainFreeDofOrdering.NumFreeDofs];
          
        //    int counter = 0;
        //    foreach ((int node, int dof, int freeDofIdx) in globalAlgebraicModel.SubdomainFreeDofOrdering.FreeDofs)
        //    {
        //        uInitialFreeDOFs_state1_array[counter] = uInitialFreeDOFs_state1_Data.Data[node, dof];
        //        counter++;

        //    }

        //    return uInitialFreeDOFs_state1_array;
        //}

            
        //    public static int ProvideIdMonitor(Model model)
        //    {
        //        //works only in the case when all the points are tottaly constrained. Note that the enumeration is beginning for the free dofs and
        //        //counts only for them. Later we may make it more general this method.
        //        var nodeid = 1;
        //        var mx = hx / 2;
        //        var my = hy / 2;
        //        var mz = hz;
        //        var Monitor = 0;
        //        for (int l = 0; l < kmax; l++)
        //        {
        //            for (int k = 0; k < jmax; k++)
        //            {
        //                for (int j = 0; j < imax; j++)
        //                {
        //                    var X = startX + j * LengthX;
        //                    var Y = startY + k * LengthY;
        //                    var Z = startZ + l * LengthZ;
        //                    var boolx = (mx == X);
        //                    var booly = (my == Y);
        //                    var boolz = (mz == Z);
        //                    bool conx = false;
        //                    bool cony = false;
        //                    bool conz = false;
        //                    //ATTENTION. THIS WORKS IF AND ONLY IF THE CONSTRAINTS ARE INPUTED IN THE MODEL LIKE THIS CONSTRAINTS[0]==STRUCTURAL.TRANSLATIONX,CONSTRAINTS[1]==STRUCTURAL.TRANSLATIONY,CONSTRAINTS[2]==STRUCTURAL.TRANSLATIONZ 
        //                    if (model.NodesDictionary[nodeid].Constraints.Count == 3)
        //                    {
        //                        conx = model.NodesDictionary[nodeid].Constraints[0].DOF.Equals(StructuralDof.TranslationX);
        //                        cony = model.NodesDictionary[nodeid].Constraints[1].DOF.Equals(StructuralDof.TranslationY);
        //                        conz = model.NodesDictionary[nodeid].Constraints[2].DOF.Equals(StructuralDof.TranslationZ);
        //                        //END OF ATTENTION
        //                    }
        //                    if (conx == false || cony == false || conz == false)
        //                    {
        //                        if (boolx && booly && boolz == true)
        //                        {
        //                            Monitor = 4 * nodeid - 2 + ((int)(hx / LengthX) + 1) * ((int)(hy / LengthY) + 1);
        //                        }
        //                    }
        //                    if (Z != 0)
        //                    {
        //                        nodeid++;
        //                    }
        //                }
        //            }
        //        }
        //        return Monitor;
        //    }
        //}
        //}

        public static IReadOnlyList<double[]> GetExpectedDisplacements()
        {
            return new double[][]
            {
                 new[] {-0.0362833756},


            };
        }
    }
}
    
    

    