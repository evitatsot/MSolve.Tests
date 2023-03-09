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
    class VonMisesLinearHardeningTestExample
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
                   
                    var gaussPointMaterials = new VonMisesMaterial3D[8];
                    double youngModulus = 800000d;
                    double poissonRatio = 1.0d / 3.0d;
                    double yieldStress = 5000d;
                     double hardeningRatio = 100000d;
                    for (int i = 0; i < gpNo; i++)
                        gaussPointMaterials[i] = new VonMisesMaterial3D(youngModulus, poissonRatio, yieldStress, hardeningRatio);
                   
                    var elementFactory = new ContinuumElement3DFactory(new VonMisesMaterial3D(800000d, 1.0d / 3.0d, 5000d, 100000d), new TransientAnalysisProperties(1, 0, 0));
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


        public static IReadOnlyList<double[]> GetExpectedDisplacements()
        {
            return new double[][]
            {
                 new[] { -0.1347},


            };
        }

    }
}
