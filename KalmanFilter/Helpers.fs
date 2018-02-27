namespace KalmanFilter
module Helpers =

    open MathNet.Numerics.LinearAlgebra

    // Corrects the given matrix to ensure it is symmetric
    let symmetrize (m:Matrix<float>) = (m + m.Transpose()).Divide(2.0)

    let idMatrix size = 
            [ for i in 0 .. size - 1 ->
                [ for j in 0 .. size - 1 ->
                    if i = j then 1.0 else 0.0 ] ] |> matrix