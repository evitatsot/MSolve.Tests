using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
    class TrescaExample
    {

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

                    var gaussPointMaterials = new Tresca[8];
                    double young = 800000d;
                    double poisson = 1.0d / 3.0d;
                    double syield = 150d;
                    for (int i = 0; i < gpNo; i++)
                        gaussPointMaterials[i] = new Tresca(young, poisson, syield);
                 
                    var elementFactory = new ContinuumElement3DFactory(new Tresca(800000d, 1.0d / 3.0d, 150.0d), new TransientAnalysisProperties(1, 0, 0));
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
            

            loads.Add(new NodalLoad(model.NodesDictionary[19], StructuralDof.TranslationZ, amount: 1 * (-375.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[22], StructuralDof.TranslationZ, amount: 1 * (-750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[25], StructuralDof.TranslationZ, amount: 1 * (-375.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[20], StructuralDof.TranslationZ, amount: 1 * (-750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[23], StructuralDof.TranslationZ, amount: 1 * (-1500.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[26], StructuralDof.TranslationZ, amount: 1 * (-750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[21], StructuralDof.TranslationZ, amount: 1 * (-375.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[24], StructuralDof.TranslationZ, amount: 1 * (-750.0d)));
            loads.Add(new NodalLoad(model.NodesDictionary[27], StructuralDof.TranslationZ, amount: 1 * (-375.0d)));


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
                 new[] {-0.00315},


            };
        }
    }
}
    
    

    
