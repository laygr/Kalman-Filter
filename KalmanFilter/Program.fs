namespace KalmanFilter

module Program =
    open System
    open DataHandling
    open FSharp.Data
    open FSharp.Data.CsvExtensions
    open XPlot.GoogleCharts
    open MathNet.Numerics.LinearAlgebra

    let test (names:string[]) data dynamics =
        // Run the E step using the calculated 'dynamics'
        let realValues, predictedValues = MarketsInteractions.onlineLearning data dynamics
        let timeSteps = [0..data.ColumnCount-1]

        let traces, logLiks = MarketsInteractions.expectationStep (MarketsInteractions.kalmanInputFor data dynamics)
        let chartForIndex index =
            // Combine line charts with original data and with
            // fitted data (for stock market with index 2)
            let size = data.ColumnCount
            let o, f =
                Seq.map (fun (real:Vector<float>) -> real.[index]) realValues |> Seq.zip timeSteps,
                Seq.map (fun (predicted:Vector<float>) -> predicted.[index]) predictedValues |> Seq.zip timeSteps

            [o; f]
            |> Chart.Combo 
            |> Chart.WithLabels ["Real"; "Predicted"]
            |> Chart.WithTitle names.[index]

        Seq.map chartForIndex [0 .. data.RowCount-1]
        |> Seq.iter Chart.Show

        //let charts = Microsoft.FSharp.Data([original; fitted])
        //let multiLayout = Layout(title = "Line Chart and a Bar Chart")
        //Figure(charts, multiLayout)

    [<EntryPoint>]
    let main argv =
        let cpiUrl = "/Users/laygr/Desktop/CPI Suecia.csv"
        let csv = CsvFile.Load(cpiUrl,hasHeaders=true)
        let names = [| "GRCP20YY"; "BECPYOY"; "NECPIYOY"; "DNCPIYOY"; "NOCPIYOY"; "Co1"; "EURSEK"; "SWCPYOY" |]
        let fullDataUnormalized = 
            prepareData2 csv names
            |> lastColumns 100
        let fullDataNormalized =
            fullDataUnormalized
            |> normalize -1.0 1.0
        let trainingData, validationData =
            fullDataUnormalized
            |> splitData
        let dynamics, traces, logliks = MarketsInteractions.main names trainingData
        
        let simpleLine name (values:float seq) =
            Chart.Line(data=Seq.mapi (fun i v -> v) values)
            |> Chart.WithTitle name

        logliks
        |> simpleLine "Log Likelihood"
        |> Chart.Show

        test names fullDataUnormalized dynamics

        0 // return an integer exit code
