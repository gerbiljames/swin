#!/usr/bin/env Rscript

library(stringr)
library(gtools)
library(effsize)

colours <- c("blue", "green", "orange", "purple", "yellow",
 "black", "red", "pink", "brown", "lightblue")

logs_dir <- "../src/PiSwarmSimulator/logs/"

beta_file_paths <- list.files(path=logs_dir, pattern="^adv_beta", full.names=TRUE)

omega_file_paths <- list.files(path=logs_dir, pattern="^adv_omega", full.names=TRUE)

beta_file_paths <- mixedsort(beta_file_paths)

omega_file_paths <- mixedsort(omega_file_paths)

beta_data <- lapply(beta_file_paths, read.csv, header=FALSE)

omega_data <- lapply(omega_file_paths, read.csv, header=FALSE)

col_names <- c("time", "beacon_distance", "centroid_distance", "lost_robots")

for (i in 1:length(beta_data)) {
	colnames(beta_data[[i]]) <- col_names
}

for (i in 1:length(omega_data)){
	colnames(omega_data[[i]]) <- col_names
}

mean_beta_data <- NULL

for (data_type in c("beacon_distance", "centroid_distance", "lost_robots")) {

	joined_beta_set <- NULL

	for (data_set in beta_data){

		trimmed_data <- data.frame(time=data_set$time, data_set[data_type])

		if (is.null(joined_beta_set)) {
			joined_beta_set <- trimmed_data
		} else {
			joined_beta_set <- merge(joined_beta_set, trimmed_data, by="time")
		}
	}

	if (is.null(mean_beta_data)) {
		mean_beta_data <- data.frame(time=joined_beta_set$time, rowMeans(joined_beta_set[,-1]))
	} else {
		new_data <- data.frame(time=joined_beta_set$time, rowMeans(joined_beta_set[,-1]))

		mean_beta_data <- merge(mean_beta_data, new_data, by="time")
	}

}

mean_omega_data <- NULL

for (data_type in c("beacon_distance", "centroid_distance", "lost_robots")) {

	joined_omega_set <- NULL

	for (data_set in omega_data){

		trimmed_data <- data.frame(time=data_set$time, data_set[data_type])

		if (is.null(joined_omega_set)) {
			joined_omega_set <- trimmed_data
		} else {
			joined_omega_set <- merge(joined_omega_set, trimmed_data, by="time")
		}
	}

	if (is.null(mean_omega_data)) {
		mean_omega_data <- data.frame(time=joined_omega_set$time, rowMeans(joined_omega_set[,-1]))
	} else {
		new_data <- data.frame(time=joined_omega_set$time, rowMeans(joined_omega_set[,-1]))

		mean_omega_data <- merge(mean_omega_data, new_data, by="time")
	}

}

colnames(mean_beta_data) <- col_names

colnames(mean_omega_data) <- col_names

pdf("figures/comparison_beacon_distance.pdf")

plot(mean_beta_data$time, mean_beta_data$beacon_distance, type="l", xlab="Time (Seconds)",
 ylab="Centroid Distance from Beacon(cm)", col=colours[[1]])

lines(mean_omega_data$time, mean_omega_data$beacon_distance, col=colours[[7]])

legend("topright", c("beta", "omega"), col=c(colours[[1]], colours[[7]]), lty=1)

rubbish <- dev.off())

pdf("figures/comparison_centroid_distance.pdf")

plot(mean_beta_data$time, mean_beta_data$centroid_distance, type="l", xlab="Time (Seconds)", 
 ylab="Mean Robot Distance from Centroid (cm)", col=colours[[1]], ylim=c(0, 80))

lines(mean_omega_data$time, mean_omega_data$centroid_distance, col=colours[[7]])

legend("right", c("beta", "omega"), col=c(colours[[1]], colours[[7]]), lty=1)

rubbish <- dev.off()

pdf("figures/comparison_lost_robots.pdf")

plot(mean_beta_data$time, mean_beta_data$lost_robots, type="l", xlab="Time (Seconds)",
 ylab="Lost Robots", col=colours[[1]], ylim=c(0, 20))

lines(mean_omega_data$time, mean_omega_data$lost_robots, col=colours[[7]])

legend("topright", c("beta", "omega"), col=c(colours[[1]], colours[[7]]), lty=1)

rubbish <- dev.off()

print("Vargha-Delaney A measure for centroid_distance")

print(VD.A(mean_beta_data$centroid_distance, mean_omega_data$centroid_distance))