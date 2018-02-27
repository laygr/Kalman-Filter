namespace KalmanFilter
module Program =
    open System
    open FSharp.Data
    open FSharp.Data.CsvExtensions
    open Data

    [<EntryPoint>]
    let main argv =
        let cpiUrl = "C:\Users\laygr\Desktop\CPI Suecia.csv"
        let csv = CsvFile.Load(cpiUrl,hasHeaders=true)
        let names = [| "GRCP20YY"; "BECPYOY"; "NECPIYOY"; "DNCPIYOY"; "NOCPIYOY"; "Co1"; "EURSEK" |]
        let trainingData, validationData =
            prepareData2 csv names
            |> normalize -1.0 1.0
            |> splitData
        let traces, logliks = MarketsInteractions.main names trainingData validationData



        0 // return an integer exit code