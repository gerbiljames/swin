#!/usr/bin/env Rscript

library(stringr)
library(gtools)

colours <- c("blue", "green", "orange", "purple", "yellow",
 "black", "red", "pink", "brown", "lightblue")

logs_dir <- "../src/PiSwarmSimulator/logs/"

files <- list.files(path=logs_dir, pattern="^omega")

file_paths <- list.files(path=logs_dir, pattern="^omega", full.names=TRUE)

files <- mixedsort(files)

file_paths <- mixedsort(file_paths)

omega_data <- lapply(file_paths, read.csv, header=FALSE)

col_names <- c("time", "distance")

for (i in 1:length(omega_data)){
	colnames(omega_data[[i]]) <- col_names
}

file_values <- sapply(files, str_sub, start=7, end=-5)

file_values <- sapply(file_values, strsplit, "_")

for (i in 1:length(file_values)){
	omega_data[[i]] <- c(omega_data[i], file_values[[i]])
}

sorted_omega_data <- list()

for (i in 1:length(omega_data)){

	sorted_index <- omega_data[[i]][[2]]

	current_index_contents <- sorted_omega_data[[sorted_index]]

	if (is.null(current_index_contents)) current_index_contents <- list()

	seed_value <- omega_data[[i]][[3]]

	data_frame <- omega_data[[i]][[1]]

	current_index_contents[[seed_value]] <- data_frame

	sorted_omega_data[[sorted_index]] <- current_index_contents

}

mean_omega_data <- list()

index_increment <- length(omega_data) / length(sorted_omega_data)

index <- 1

for (data_set in sorted_omega_data) {

	current_joined_set <- NULL

	for (data in data_set) {
		if (is.null(current_joined_set)) {
				current_joined_set <- data
			} else {
				current_joined_set <- merge(current_joined_set, data, by="time")
			}
	}

	mean_data_set <- data.frame(time=current_joined_set$time, mean_distance=rowMeans(current_joined_set[,-1]))

	mean_omega_data[[file_values[[index]][[1]]]] <- mean_data_set

	index <- index + index_increment

}

pdf("figures/omega_calibration.pdf")

plot(mean_omega_data[[1]], xlim=c(0, 15000), ylim=c(0, 500), type="n",
	xlab="Time (Seconds)", ylab="Centroid Distance from Beacon(cm)")

legend_text <- list()

index <- 1

for (i in 1:length(mean_omega_data)) {
	lines(mean_omega_data[[i]], col=colours[[i]])
	legend_text[[i]] <- file_values[[index]][[1]]

	index <- index + index_increment
}

legend("topright", col=colours, legend=legend_text, lty=1, title="omega")

rubbish <- dev.off()